import time, random, json, sys

FS = 100  # Hz

def sample(device):
    return {
        "timestamp": time.time(),
        "device": device,
        "accel": {
            "x": round(random.uniform(-2, 2), 3),
            "y": round(random.uniform(-2, 2), 3),
            "z": round(random.uniform(8.5, 10.5), 3)
        },
        "gyro": {
            "x": round(random.uniform(-1, 1), 3),
            "y": round(random.uniform(-1, 1), 3),
            "z": round(random.uniform(-1, 1), 3)
        }
    }

while True:
    frame = [sample("wrist"), sample("ankle")]
    sys.stdout.write(json.dumps(frame) + "\n")
    sys.stdout.flush()
    time.sleep(1/FS)
