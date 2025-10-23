"""
ble_receiver.py
---------------
Multi-device BLE receiver for Arduino Nano 33 IoT movement nodes.

- Scans by advertised name (AMS-Wrist, AMS-Ankle)
- Cross-platform Bleak usage (Windows/macOS vs Linux/Raspberry Pi)
- Subscribes to LABEL (…0006, short ASCII <=20 bytes)
- Optionally enables and subscribes to RAW (…0005, 12-byte binary sample)
- Sends START @ SAMPLE_HZ after subscribing
- Queues packets as (node_label, stream_type, payload_bytes)

Usage (standalone):
  python ble_receiver.py

Integration:
  rx = BLEReceiver(asyncio.Queue(), enable_raw=True)
  await rx.connect_all()
  ... consume rx.queue ...
"""

import asyncio
import platform
import struct
from typing import Optional, Dict
from bleak import BleakClient, BleakScanner, BleakError

# --- UUIDs must match the Arduino sketch ---
UUID_CHAR_CTRL  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Write (text: start/stop/raw:on/off; or binary <BH>)
UUID_CHAR_RAW   = "6e400005-b5a3-f393-e0a9-e50e24dcca9e"  # Notify (12 bytes: <6h> ax,ay,az,gx,gy,gz in 1e-3 g/dps)
UUID_CHAR_LABEL = "6e400006-b5a3-f393-e0a9-e50e24dcca9e"  # Notify (ASCII <=20: e.g., "WALK,1.22,0.42")

# --- Nodes you expect (add/remove as needed) ---
TARGETS: Dict[str, str] = {
    "wrist": "AMS-Wrist",
    "ankle": "AMS-Ankle",
}

SAMPLE_HZ = 100          # START rate to request
ENABLE_RAW_DEFAULT = True  # set False if you only want LABEL

POST_CONNECT_GRACE_S = 0.6  # small delay so Windows GATT cache is ready

class BLEReceiver:
    def __init__(self, queue: asyncio.Queue, enable_raw: bool = ENABLE_RAW_DEFAULT, sample_hz: int = SAMPLE_HZ):
        self.queue = queue
        self.enable_raw = enable_raw
        self.sample_hz = sample_hz
        self.clients: Dict[str, BleakClient] = {}
        self._running = False

    # -------- helper: platform-safe client ctor ----------
    def _make_client(self, address: str) -> BleakClient:
        if platform.system() == "Linux":
            return BleakClient(address, timeout=15.0, address_type="random", adapter="hci0")
        return BleakClient(address, timeout=15.0)

    # -------- scan by name (avoid stale MACs on Windows) ----------
    async def _resolve_by_name(self, name_hint: str) -> Optional[str]:
        print(f"[SCAN] Looking for '{name_hint}' …")
        devs = await BleakScanner.discover(timeout=5.0)
        dev = next((d for d in devs if d.name and name_hint in d.name), None)
        if not dev:
            print(f"[WARN] Not found: {name_hint}. Saw: {[d.name for d in devs if d.name]}")
            return None
        print(f"[FOUND] {dev.name} @ {dev.address}")
        return dev.address

    # -------- notify callbacks ----------
    def _cb_raw(self, node):
        def inner(_, data: bytearray):
            if not data or len(data) != 12:
                return
            self.queue.put_nowait((node, "raw", bytes(data)))
        return inner

    def _cb_label(self, node):
        def inner(_, data: bytearray):
            if not data:
                return
            self.queue.put_nowait((node, "label", bytes(data)))
        return inner

    # -------- connect one node ----------
    async def _connect_node(self, node_label: str):
        name_hint = TARGETS[node_label]
        addr = await self._resolve_by_name(name_hint)
        if not addr:
            return

        client = self._make_client(addr)

        # retry connect a few times (Windows is sometimes flaky)
        for attempt in range(3):
            try:
                await client.connect()
                await asyncio.sleep(POST_CONNECT_GRACE_S)
                if not client.is_connected:
                    raise BleakError("connect() returned but not connected")
                break
            except Exception as e:
                print(f"[WARN] {node_label} connect attempt {attempt+1}/3 failed: {e}")
                if attempt == 2:
                    print(f"[ERR] Giving up {node_label}")
                    return
                await asyncio.sleep(1.0)

        self.clients[node_label] = client
        print(f"[OK] Connected -> {name_hint}")

        # Subscribe to LABEL first (works everywhere)
        await client.start_notify(UUID_CHAR_LABEL, self._cb_label(node_label))

        # Optionally enable raw and subscribe
        if self.enable_raw:
            try:
                await client.write_gatt_char(UUID_CHAR_CTRL, b"raw:on", response=True)
                await client.start_notify(UUID_CHAR_RAW, self._cb_raw(node_label))
                print(f"[RAW] {node_label}: enabled + subscribed")
            except Exception as e:
                print(f"[WARN] {node_label}: could not enable RAW -> {e}")

        # Finally send START @ sample_hz (binary control <BH>)
        try:
            await client.write_gatt_char(UUID_CHAR_CTRL, struct.pack("<BH", 1, self.sample_hz), response=True)
            print(f"[START] {node_label}: {self.sample_hz} Hz")
        except Exception as e:
            print(f"[ERR] {node_label}: START failed -> {e}")

    # -------- public: connect all known targets ----------
    async def connect_all(self):
        await asyncio.gather(*(self._connect_node(label) for label in TARGETS.keys()))

    # -------- public: disconnect all ----------
    async def disconnect_all(self):
        for label, client in list(self.clients.items()):
            try:
                try:
                    await client.write_gatt_char(UUID_CHAR_CTRL, struct.pack("<BH", 0, 0), response=True)
                except Exception:
                    pass
                try:
                    await client.stop_notify(UUID_CHAR_RAW)
                except Exception:
                    pass
                try:
                    await client.stop_notify(UUID_CHAR_LABEL)
                except Exception:
                    pass
                await client.disconnect()
                print(f"[OK] Disconnected {label}")
            except Exception as e:
                print(f"[WARN] Disconnect {label} failed: {e}")

    # -------- optional demo loop ----------
    async def run(self):
        self._running = True
        while self._running:
            try:
                node, stream, payload = await asyncio.wait_for(self.queue.get(), timeout=5.0)
                if stream == "label":
                    s = payload.decode("utf-8", "ignore").strip("\x00").strip()
                    print(f"[{node}][LABEL] {s}")
                else:
                    # raw 12 bytes -> 6x int16
                    ax, ay, az, gx, gy, gz = struct.unpack("<6h", payload)
                    print(f"[{node}][RAW] ax:{ax} ay:{ay} az:{az} gx:{gx} gy:{gy} gz:{gz}")
            except asyncio.TimeoutError:
                print("[INFO] Waiting for data...")
            except Exception as e:
                print("[ERR] Queue error:", e)

    async def stop(self):
        self._running = False
        await self.disconnect_all()


# ---------- allow running standalone ----------
if __name__ == "__main__":
    async def main():
        q = asyncio.Queue()
        rx = BLEReceiver(q, enable_raw=True, sample_hz=SAMPLE_HZ)
        await rx.connect_all()
        try:
            await rx.run()
        except KeyboardInterrupt:
            print("Stopping…")
        finally:
            await rx.stop()

    asyncio.run(main())
