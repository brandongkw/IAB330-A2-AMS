import asyncio
from bleak import BleakScanner, BleakClient

UUID_CHAR_CTRL = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
UUID_CHAR_BIN  = "6e400006-b5a3-f393-e0a9-e50e24dcca9e"

TARGET_NAME = "AMS-Wrist"   # or "AMS-Ankle"

async def main():
    print("Scanning 5s...")
    devices = await BleakScanner.discover(timeout=5.0)
    target = next((d for d in devices if d.name and TARGET_NAME in d.name), None)
    if not target:
        print("Not found. Found names:", [d.name for d in devices if d.name])
        return

    print("Connecting to", target.name, target.address)
    # Simple, portable client ctor (works on Windows/macOS and Pi)
    async with BleakClient(target, timeout=15.0) as c:
        print("Connected:", c.is_connected)
        # small grace so Windows GATT db is ready
        await asyncio.sleep(0.5)

        # turn on binary stream & subscribe BEFORE start
        await c.write_gatt_char(UUID_CHAR_CTRL, b"bin:on", response=True)

        def on_notify(_, data: bytearray):
            print("Notify len:", len(data))

        await c.start_notify(UUID_CHAR_BIN, on_notify)

        # start @100 Hz (legacy 3-byte control: cmd=1, rate=100)
        import struct
        await c.write_gatt_char(UUID_CHAR_CTRL, struct.pack("<BH", 1, 100), response=True)

        print("Receiving for 10s...")
        await asyncio.sleep(10.0)

        # stop
        await c.write_gatt_char(UUID_CHAR_CTRL, struct.pack("<BH", 0, 0), response=True)
        await c.stop_notify(UUID_CHAR_BIN)
        print("Done")

asyncio.run(main())
