import serial
import time
import struct
import json
import zmq

SERIAL_PORT = '/dev/imu'
BAUD_RATE = 115200

REFRESH_INTERVAL = 0.01
OVERWRITE_OUTPUT = True

OPR_MODE_REG = 0x3D
EULER_H_LSB = 0x1A

CONFIG_MODE = 0x00
NDOF_MODE = 0x0C

def send_command(ser, cmd, timeout=0.05):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    ser.write(cmd)
    start = time.time()
    buf = bytearray()
    while len(buf) < 2 and (time.time() - start) < timeout:
        b = ser.read(1)
        if b:
            buf += b
    if len(buf) < 2:
        return None
    payload_len = buf[1]
    total_len = 2 + payload_len
    while len(buf) < total_len and (time.time() - start) < timeout:
        chunk = ser.read(total_len - len(buf))
        if chunk:
            buf += chunk
    if not buf:
        return None
    return bytes(buf)

def set_operation_mode(ser, mode):
    cmd = bytearray([0xAA, 0x00, OPR_MODE_REG, 0x01, mode])
    response = send_command(ser, cmd, timeout=0.1)
    if response == bytes([0xEE, 0x01]):
        print("Operation mode set successfully.")
    else:
        print(f"Warning: unexpected set mode response: {response}")

def read_euler_angles(ser):
    cmd = bytearray([0xAA, 0x01, EULER_H_LSB, 0x06])
    response = send_command(ser, cmd, timeout=0.04)
    if not response:
        return None
    if len(response) >= 8 and response[0] == 0xBB and response[1] == 0x06:
        data = response[2:8]
        try:
            heading, roll, pitch = struct.unpack('<hhh', data)
            return heading / 16.0, roll / 16.0, pitch / 16.0
        except struct.error:
            return None
    return None

try:
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) as ser:
        print("Connected to BNO055 via UART.")
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.setsockopt(zmq.SNDHWM, 1)
        try:
            socket.setsockopt(zmq.IMMEDIATE, 1)
        except Exception:
            pass
        socket.setsockopt(zmq.LINGER, 0)
        zmq_addr = "tcp://*:5001"
        socket.bind(zmq_addr)
        time.sleep(0.02)
        print(f"ZMQ PUB bound to {zmq_addr}")
        set_operation_mode(ser, CONFIG_MODE)
        time.sleep(0.02)
        set_operation_mode(ser, NDOF_MODE)
        time.sleep(0.3)
        import sys
        while True:
            angles = read_euler_angles(ser)
            if angles:
                heading, _roll, _pitch = angles
                heading = heading % 360.0
                if OVERWRITE_OUTPUT:
                    print(f"Heading: {heading:.2f}°", end='\r', flush=True)
                else:
                    print(f"Heading: {heading:.2f}°")
                payload = json.dumps({
                    "ts": time.time(),
                    "heading": round(heading, 2)
                })
                try:
                    socket.send_multipart([b"imu", payload.encode('utf-8')], zmq.NOBLOCK)
                except zmq.Again:
                    pass
                except Exception as e:
                    print(f"ZMQ publish error: {e}", file=sys.stderr)
            else:
                if not OVERWRITE_OUTPUT:
                    print("Waiting for valid heading...")
            time.sleep(REFRESH_INTERVAL)
        socket.close()
        context.term()
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Stopped.")