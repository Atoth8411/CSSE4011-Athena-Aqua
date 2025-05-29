import asyncio
import struct
from bleak import BleakClient, BleakScanner

# Your Zephyr BLE device details
DEVICE_NAME = "Zephyr_GATT_Server"
ANGLE_UUID = "abcdabcd-abcd-abcd-abcd-abcdabcdef01"

async def find_device():
    print("[INFO] Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            print(f"[INFO] Found target device: {d.name} ({d.address})")
            return d.address
    print("[ERROR] Device not found")
    return None

async def send_angle(address, angle):
    print(f"[INFO] Connecting to {address}...")
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("[ERROR] Failed to connect")
            return
        print("[INFO] Connected, sending angle...")

        # Clamp and pack the angle
        angle = max(0, min(180, angle))
        packed = struct.pack("<h", angle)
        await client.write_gatt_char(ANGLE_UUID, packed)
        print(f"[INFO] Sent angle: {angle}")

async def main():
    address = await find_device()
    if not address:
        return

    # Example test angles
    for angle in [45, 90, 135, 180, 0]:
        await send_angle(address, angle)
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
