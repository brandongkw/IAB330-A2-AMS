import os, sys, json, time, re, struct, asyncio
from datetime import datetime
from inference import InferenceEngine
from mongo_upload import MongoSink

# ---------- .env loader (no external deps) ----------
def load_env_file(path: str = ".env", override: bool = False) -> None:
    if not os.path.isfile(path):
        return
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            k = k.strip()
            v = v.strip().strip("'").strip('"')
            v = re.sub(r"\$\{([^}]+)\}", lambda m: os.environ.get(m.group(1), ""), v)
            if override or k not in os.environ:
                os.environ[k] = v

load_env_file(".env")
# ---------------------------------------------------------

_PRINTABLE = set(chr(i) for i in range(32, 127))
_NUM_RE = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')

def _sanitize_label_bytes(b: bytes) -> str:
    s = b.decode("utf-8", "ignore")
    s = s.replace("\x00", "").replace("\r", "").replace("\n", "")
    s = "".join(ch for ch in s if ch in _PRINTABLE)
    return s.strip()

def _extract_floats(s: str):
    return [float(x) for x in _NUM_RE.findall(s)]

# ------------------ env & settings ------------------
MODE = os.environ.get("MODE", "BLE").upper()
FS_HZ = int(os.environ.get("FS_HZ", "100"))
WINDOW_SECONDS = float(os.environ.get("WINDOW_SECONDS", "2"))
BATCH_UPLOAD_SECONDS = int(os.environ.get("BATCH_UPLOAD_SECONDS", "5"))
SAVE_RAW = int(os.environ.get("SAVE_RAW", "0"))

MONGO_URI = os.environ.get("MONGO_URI", "")
MONGO_DB  = os.environ.get("MONGO_DB", "HAR_Project")
SENSOR_COLLECTION = os.environ.get("MONGO_COLLECTION", "SensorData")
PRED_COLLECTION   = os.environ.get("MONGO_PRED_COLLECTION", "Predictions")
MODEL_PATH = os.environ.get("MODEL_PATH", "final_HAR_model.joblib")

os.makedirs("data/backlog", exist_ok=True)
os.makedirs("logs", exist_ok=True)

sensor_sink = MongoSink(MONGO_URI, MONGO_DB, SENSOR_COLLECTION) if MONGO_URI else None
pred_sink   = MongoSink(MONGO_URI, MONGO_DB, PRED_COLLECTION)   if MONGO_URI else None
print(f"[cfg] Sensor sink: {'ON' if sensor_sink else 'OFF'} -> {SENSOR_COLLECTION}")
print(f"[cfg] Pred sink:   {'ON' if pred_sink else 'OFF'}   -> {PRED_COLLECTION}")
engine = InferenceEngine(MODEL_PATH)

buffer = {"wrist": [], "ankle": []}   # each item: {"accel":{"x":..,"y":..,"z":..}, "gyro":{...}}
sensor_batch = []                     # raw + label/speed summaries
pred_batch   = []                     # predictions only
last_upload  = time.time()
window_size  = max(1, int(WINDOW_SECONDS * FS_HZ))

# track last label-derived stats, and last time we saw RAW
last_stats = {
    "wrist": {"mean": None, "std": None, "t": 0.0},
    "ankle": {"acc_mag": None, "intensity": None, "t": 0.0},
}
last_raw_ts = {"wrist": 0.0, "ankle": 0.0}
DEBUG_PRINT_EVERY = 1.0  # seconds
_last_dbg = 0.0


def _to_vec3_dict(v):
    """Accept [x,y,z] or dict and return dict form."""
    if isinstance(v, dict):
        return {"x": float(v.get("x", 0)), "y": float(v.get("y", 0)), "z": float(v.get("z", 0))}
    # assume list/tuple
    return {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])}

def add_sample(sample):
    """sample: {"device": "wrist/ankle", "accel":[x,y,z] or dict, "gyro":[x,y,z] or dict}"""
    limb = sample.get("device", "wrist")
    if limb not in buffer:
        return
    # normalize shape for feature extractor
    accel = _to_vec3_dict(sample["accel"])
    gyro  = _to_vec3_dict(sample["gyro"])
    norm = {"accel": accel, "gyro": gyro}
    buffer[limb].append(norm)
    last_raw_ts[limb] = time.time()

    # keep buffer reasonable
    max_keep = window_size * 3
    if len(buffer[limb]) > max_keep:
        buffer[limb] = buffer[limb][-max_keep:]

    if SAVE_RAW:
        sensor_batch.append({
            "type": "raw",
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "device": limb,
            "accel": [accel["x"], accel["y"], accel["z"]],
            "gyro":  [gyro["x"],  gyro["y"],  gyro["z"]],
        })

def maybe_predict():
    global _last_dbg
    # debug heartbeat: show buffer lengths + RAW recency
    now = time.time()
    if now - _last_dbg >= DEBUG_PRINT_EVERY:
        print(f"[buf] wrist={len(buffer['wrist'])}/{window_size}  ankle={len(buffer['ankle'])}/{window_size}  "
              f"raw_age(w)={now-last_raw_ts['wrist']:.2f}s  raw_age(a)={now-last_raw_ts['ankle']:.2f}s")
        _last_dbg = now

    enough_wrist = len(buffer["wrist"]) >= window_size
    enough_ankle = len(buffer["ankle"]) >= window_size

    # 1) Normal path: window ready -> real features -> model
    if enough_wrist or enough_ankle:
        window = {}
        if enough_wrist: window["wrist"] = buffer["wrist"][-window_size:]
        if enough_ankle: window["ankle"] = buffer["ankle"][-window_size:]
        label, proba = engine.predict_window(window)
        ts = datetime.now().strftime("%H:%M:%S")
        if proba is not None:
            print(f"[{ts}] Predicted: {label}  (p={float(max(proba)):.2f})", flush=True)
        else:
            print(f"[{ts}] Predicted: {label}", flush=True)
    else:
        # 2) Fallback: no RAW window yet; if RAW is stale, synthesize a best-effort prediction
        raw_stale = (now - max(last_raw_ts['wrist'], last_raw_ts['ankle'])) >= 1.0
        have_wrist = last_stats["wrist"]["std"] is not None
        have_ankle = last_stats["ankle"]["intensity"] is not None

        if raw_stale and (have_wrist or have_ankle):
            # Simple heuristic aligned with _FallbackModel thresholds
            std_w = last_stats["wrist"]["std"] or 0.0
            std_a = last_stats["ankle"]["intensity"] or 0.0
            m = max(std_w, std_a)
            if m < 0.15:
                label, proba = "Idle", None
            elif m < 0.50:
                label, proba = "Walking", None
            else:
                label, proba = "Running", None
            ts = datetime.now().strftime("%H:%M:%S")
            print(f"[{ts}] Predicted*(synthetic): {label}  (σ_w={std_w:.2f}, I_a={std_a:.2f})", flush=True)
        else:
            return  # nothing to do yet

    # store in Predictions collection
    doc = {
        "type": "prediction",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "label": label,
    }
    if proba is not None:
        doc["probs"] = [float(x) for x in proba]
    pred_batch.append(doc)
    print(f"[pred] queued -> {label} (batch size={len(pred_batch)})", flush=True)

    # OPTIONAL: eager upload for demos – set PRED_UPLOAD_EAGER=1 in .env
    if os.environ.get("PRED_UPLOAD_EAGER", "0") == "1" and pred_sink:
        try:
            pred_sink.insert_batch(pred_batch)
            pred_sink.flush_backlog()
            print(f"[mongo] Uploaded {len(pred_batch)} docs to {PRED_COLLECTION}")
            pred_batch.clear()
        except Exception as e:
            print(f"[mongo] upload error (pred-eager): {e}")

def maybe_upload():
    """Flush both SensorData and Predictions batches on the timer.
       Each sink is isolated so one failing never blocks the other."""
    global last_upload, sensor_batch, pred_batch

    # gate on interval
    if (time.time() - last_upload) < BATCH_UPLOAD_SECONDS:
        return

    # track what we did for clear logging
    uploaded = {"sensor": 0, "pred": 0}
    errors = []

    # ---- SensorData ----
    if sensor_sink and sensor_batch:
        to_send = sensor_batch
        try:
            sensor_sink.insert_batch(to_send)
            sensor_sink.flush_backlog()
            uploaded["sensor"] = len(to_send)
            # clear only after successful path or backlog write
            sensor_batch = []
        except Exception as e:
            errors.append(f"sensor: {e}")
            # keep batch; it will be retried next tick (and also written to backlog by sink)

    # ---- Predictions ----
    if pred_sink and pred_batch:
        to_send = pred_batch
        try:
            pred_sink.insert_batch(to_send)
            pred_sink.flush_backlog()
            uploaded["pred"] = len(to_send)
            pred_batch = []
        except Exception as e:
            errors.append(f"pred: {e}")

    # bump the timer regardless, so we don’t tight-loop
    last_upload = time.time()

    # concise logs so you can see both attempted
    if uploaded["sensor"]:
        print(f"[mongo] Uploaded {uploaded['sensor']} docs to {SENSOR_COLLECTION}")
    if uploaded["pred"]:
        print(f"[mongo] Uploaded {uploaded['pred']} docs to {PRED_COLLECTION}")
    if errors:
        print("[mongo] upload error(s): " + " | ".join(errors))

def run_mock():
    from subprocess import Popen, PIPE
    p = Popen([sys.executable, "mock_stream.py"], stdout=PIPE, text=True, bufsize=1)
    for line in p.stdout:
        try:
            frame = json.loads(line.strip())
            for s in frame:
                add_sample(s)
        except Exception:
            continue
        maybe_predict()
        maybe_upload()

def run_ble():
    from ble_receiver import BLEReceiver
    import asyncio, struct

    q = asyncio.Queue()
    rx = BLEReceiver(q, enable_raw=True, sample_hz=FS_HZ)

    async def producer():
        # Connect both nodes and just keep the connection alive.
        await rx.connect_all()
        try:
            while True:
                await asyncio.sleep(1.0)  # keep the task alive; callbacks fill q
        finally:
            await rx.stop()

    async def consumer():
        while True:
            try:
                node, stream, payload = await asyncio.wait_for(q.get(), timeout=0.05)
            except asyncio.TimeoutError:
                # advance the pipeline even when no packet arrives
                maybe_predict()
                maybe_upload()
                continue

            if stream == "raw":
                if len(payload) != 12:
                    continue
                ax, ay, az, gx, gy, gz = struct.unpack("<6h", payload)
                sample = {
                    "device": node,
                    "accel": [ax/1000.0, ay/1000.0, az/1000.0],
                    "gyro":  [gx/1000.0, gy/1000.0, gz/1000.0],
                }
                add_sample(sample)
                last_raw_ts[node] = time.time()

            elif stream == "label":
                s = _sanitize_label_bytes(payload)
                if not s:
                    continue

                now = time.time()

                if s.startswith("SPD"):   # ankle: SPD,<accMag_g>,<intensity>
                    nums = _extract_floats(s)
                    if len(nums) >= 2:
                        acc_mag = nums[0]; intensity = nums[1]
                        last_stats["ankle"].update({"acc_mag": acc_mag, "intensity": intensity, "t": now})
                        sensor_batch.append({
                            "type": "speed",
                            "timestamp": datetime.utcnow().isoformat() + "Z",
                            "device": node,
                            "acc_mag_g": acc_mag,
                            "intensity": intensity,
                            "raw": s,
                        })
                else:                      # wrist: LABEL,<mean>,<std>
                    parts = s.split(",", 1)
                    label = parts[0].strip() if parts else "UNKNOWN"
                    nums = _extract_floats(s)
                    mean = nums[0] if len(nums) >= 1 else None
                    std  = nums[1] if len(nums) >= 2 else None
                    if (mean is not None) or (std is not None):
                        last_stats["wrist"].update({"mean": mean, "std": std, "t": now})
                    doc = {
                        "type": "label",
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "device": node,
                        "label": label,
                        "raw": s,
                    }
                    if mean is not None: doc["mean"] = mean
                    if std  is not None: doc["std"]  = std
                    sensor_batch.append(doc)


            # run ML & uploads continuously
            maybe_predict()
            maybe_upload()

    async def main():
        prod = asyncio.create_task(producer())
        cons = asyncio.create_task(consumer())
        try:
            await asyncio.gather(prod, cons)
        except asyncio.CancelledError:
            pass
        finally:
            prod.cancel()
            cons.cancel()
            await rx.stop()

    asyncio.run(main())


if __name__ == "__main__":
    print(f"Starting Pi Central in {MODE} mode", flush=True)
    if MODE == "BLE":
        run_ble()
    else:
        run_mock()
