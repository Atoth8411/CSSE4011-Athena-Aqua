import asyncio
import socket
import struct
from bleak import BleakScanner, BleakClient

# UUIDs
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
ANGLE_UUID = "abcdabcd-abcd-abcd-abcd-abcdabcdef01"

# Target device name advertised by Zephyr
DEVICE_NAME = "Zephyr_GATT_Server"

# UDP Server Settings
UDP_IP = "127.0.0.1"
UDP_PORT = 9999


def track_logic(x1, x2, y1, y2):
    angle = 0
    step = 6

    frame_cen = 320

    x_error = x1 + (x2 - x1)/2

    direction = x_error - frame_cen
    if (abs(direction) < 150):
        step = 1
    if (direction < -55):
        angle = step
    elif (direction > 75):
        angle = -step
    else:
        angle = 0
    return angle



async def setup_ble():
    print(f"[INFO] Scanning for BLE device: {DEVICE_NAME}...")
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            print(f"[INFO] Found BLE device: {d.name} ({d.address})")
            client = BleakClient(d.address)
            await client.connect()
            if client.is_connected:
                print("[INFO] BLE connected")
                return client
    print("[ERROR] BLE device not found")
    return None

async def safe_write(client: BleakClient, uuid: str, data: bytes):
    try:
        if not client.is_connected:
            print("[WARN] BLE disconnected. Reconnecting...")
            await client.connect()
        await client.write_gatt_char(uuid, data)
        print(f"[INFO] Sent angle (safe_write): {struct.unpack('<h', data)[0]}")
    except Exception as e:
        print(f"[BLE ERROR] Failed during safe_write: {e}")

async def receive_and_send_angle(client: BleakClient):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[INFO] Listening on {UDP_IP}:{UDP_PORT} for coordinates...")

    while True:
        try:
            data, _ = sock.recvfrom(8)
        except socket.timeout:
            continue

        if len(data) != 8:
            print("[WARN] Incorrect data size")
            continue

        x1, x2, y1, y2 = struct.unpack("<hhhh", data)
        print(f"[INFO] Received: x1={x1}, x2={x2}, y1={y1}, y2={y2}")

        angle = track_logic(x1, x2, y1, y2)

        packed_angle = struct.pack("<h", angle)
        await safe_write(client, ANGLE_UUID, packed_angle)

async def main():
    client = await setup_ble()
    if client:
        await receive_and_send_angle(client)

if __name__ == "__main__":
    asyncio.run(main())
