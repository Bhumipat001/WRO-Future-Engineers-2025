#!/usr/bin/env python3
import asyncio
import sys
import time
import threading
import queue
import json
import zmq
import serial

ZMQ_PORT_LIDAR = 5000
ZMQ_PORT_IMU = 5001

ESP32_PORT = "/dev/esp32"
ESP32_BAUD = 115200

FRONT_THRESHOLD = 1300
FIRST_FRONT_THRESHOLD = 1200 
FORWARD_SPEED_A = 100
FORWARD_SPEED_B = 100
TURN_SPEED_A = 100
TURN_SPEED_B = 100
SERVO_FORWARD = 81
SERVO_LEFT = 45
SERVO_RIGHT = 135 

FIRST_TURN_ANGLE = 75   
TURN_ANGLE = 84         
TURN_OFFSET = 0

PARK_OFFSET = 1050       
PARK_TOLERANCE = 100       
PARK_MAX_SPEED = 100     
PARK_MIN_SPEED = 50     
PARK_KP = 0.08       
PARK_CMD_HZ = 10      

SAFE_MIN_FRONT = 120      

ZMQ_RECV_TIMEOUT_MS = 750
SERIAL_READ_TIMEOUT = 0.2
SERIAL_CMD_RESPONSE_WAIT = 0.2

DEBUG = False

def log(msg, category="info"):
    if DEBUG:
        print(f"[{category}] {msg}")
    elif category == "info":
        print(msg)

_serial_cmd_queue = queue.Queue()
_stop_event = threading.Event()

def zmq_listener_thread(loop, sensor_queue, stop_event):
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    try:
        sock.connect(f"tcp://localhost:{ZMQ_PORT_LIDAR}")
    except Exception:
        pass
    try:
        sock.connect(f"tcp://localhost:{ZMQ_PORT_IMU}")
    except Exception:
        pass
    sock.setsockopt(zmq.RCVTIMEO, ZMQ_RECV_TIMEOUT_MS)

    try:
        while not stop_event.is_set():
            try:
                msg = sock.recv_string(flags=0)
            except zmq.Again:
                continue
            except Exception as e:
                loop.call_soon_threadsafe(sensor_queue.put_nowait, {"__error__": str(e)})
                break

            payload = msg
            for pfx in ("imu ", "lidar ", "imu:", "lidar:"):
                if msg.startswith(pfx):
                    payload = msg[len(pfx):].strip()
                    break
            try:
                data = json.loads(payload)
            except Exception:
                data = {"__raw__": payload}

            loop.call_soon_threadsafe(sensor_queue.put_nowait, data)
    finally:
        try:
            sock.close()
            ctx.term()
        except Exception:
            pass
        log("ZMQ listener thread exiting", "zmq")

def serial_worker_thread(loop, serial_in_queue, cmd_queue, stop_event):
    try:
        ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=SERIAL_READ_TIMEOUT)
        time.sleep(1.5)
    except Exception as e:
        loop.call_soon_threadsafe(serial_in_queue.put_nowait, {"__serial_error__": str(e)})
        return

    log(f"Serial opened {ESP32_PORT} @ {ESP32_BAUD}", "serial")

    try:
        while not stop_event.is_set():
            try:
                cmd, fut = cmd_queue.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd is not None:
                try:
                    ser.reset_output_buffer()
                    ser.write((cmd + "\n").encode())
                    ser.flush()
                    time.sleep(SERIAL_CMD_RESPONSE_WAIT)
                    resp = b""
                    n = ser.in_waiting
                    if n:
                        resp = ser.read(n)
                    resp_decoded = resp.decode(errors="ignore").strip()
                except Exception as e:
                    resp_decoded = f"SER_ERR:{e}"

                def _set_result():
                    if not fut.done():
                        fut.set_result(resp_decoded)
                loop.call_soon_threadsafe(_set_result)

            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    loop.call_soon_threadsafe(serial_in_queue.put_nowait, line)
            except Exception:
                pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        log("Serial worker thread exiting", "serial")

async def send_command(cmd: str, timeout: float = 3.0) -> str:
    loop = asyncio.get_running_loop()
    fut = loop.create_future()
    _serial_cmd_queue.put((cmd, fut))
    try:
        return await asyncio.wait_for(fut, timeout=timeout)
    except asyncio.TimeoutError:
        if not fut.done():
            fut.set_result("")
        return ""

async def drive(a: int, b: int, servo: float | None = None):
    if servo is None:
        return await send_command(f"MA:{int(a)},MB:{int(b)}")
    return await send_command(f"MA:{int(a)},MB:{int(b)},S:{servo}")

async def brake(servo_center: float = None):
    if servo_center is None:
        return await send_command("MA:0,MB:0")
    return await send_command(f"MA:0,MB:0,S:{servo_center}")

async def process_queue():
    loop = asyncio.get_running_loop()
    sensor_queue = asyncio.Queue()
    serial_in_queue = asyncio.Queue()

    t_zmq = threading.Thread(target=zmq_listener_thread, args=(loop, sensor_queue, _stop_event), daemon=True)
    t_serial = threading.Thread(target=serial_worker_thread, args=(loop, serial_in_queue, _serial_cmd_queue, _stop_event), daemon=True)
    t_zmq.start()
    t_serial.start()

    last_front = None
    last_heading = None
    started = False
    last_len = 0

    start_front = None
    target_front = None
    turn_count = 0
    parking_mode = False

    turn_direction = None
    is_ccw = True

    initial_heading_at_start = None
    cumulative_target_heading = None
    first_turn_pending = False

    last_cmd_time = 0.0
    cmd_period = 1.0 / PARK_CMD_HZ
    last_drive_vals = (None, None)

    def shortest_angle_diff(target, current):
        d = (target - current + 540) % 360 - 180
        return d

    async def sensor_drain_task():
        nonlocal last_front, last_heading, turn_direction, is_ccw
        while True:
            data = await sensor_queue.get()
            if isinstance(data, dict):
                if "front_mm" in data:
                    last_front = data.get("front_mm")
                if "heading" in data:
                    last_heading = data.get("heading")
                    if not started:
                        if last_heading is not None:
                            if abs(last_heading - 0) <= 10 or abs(last_heading - 360) <= 10:
                                turn_direction = "CCW"
                                is_ccw = True
                            elif abs(last_heading - 180) <= 10:
                                turn_direction = "CW"
                                is_ccw = False
            await asyncio.sleep(0)

    async def serial_drain_task():
        nonlocal started, start_front, turn_count, parking_mode, target_front
        nonlocal initial_heading_at_start, cumulative_target_heading, first_turn_pending, is_ccw, turn_direction
        while True:
            line = await serial_in_queue.get()
            if isinstance(line, str):
                log(f"SERIAL -> {line}", "serial")
                if line.strip() == "Start":
                    if not started:
                        started = True
                        parking_mode = False
                        turn_count = 0
                        start_front = last_front
                        target_front = None
                        if last_heading is not None:
                            initial_heading_at_start = last_heading
                            if abs(initial_heading_at_start - 0) <= 10 or abs(initial_heading_at_start - 360) <= 10:
                                turn_direction = "CCW"
                                is_ccw = True
                            elif abs(initial_heading_at_start - 180) <= 10:
                                turn_direction = "CW"
                                is_ccw = False
                            else:
                                turn_direction = "CCW"
                                is_ccw = True
                            if is_ccw:
                                cumulative_target_heading = (initial_heading_at_start - (FIRST_TURN_ANGLE + TURN_OFFSET)) % 360
                            else:
                                cumulative_target_heading = (initial_heading_at_start + (FIRST_TURN_ANGLE + TURN_OFFSET)) % 360
                            first_turn_pending = True
                            print(f"\nReceived 'Start'. Start front = {start_front} mm, initial heading = {initial_heading_at_start}°, Direction set to {turn_direction}. First target heading = {cumulative_target_heading}°")
                        else:
                            initial_heading_at_start = None
                            cumulative_target_heading = None
                            first_turn_pending = True
                            print(f"\nReceived 'Start'. Start front = {start_front} mm, IMU heading not available at Start. Falling back to reading heading when turn begins.")
                        await drive(FORWARD_SPEED_A, FORWARD_SPEED_B, SERVO_FORWARD)
                    else:
                        print("\nReceived 'Start' again -> STOPPING car and waiting...")
                        await brake(SERVO_FORWARD)
                        started = False
                        parking_mode = False
                        turn_count = 0
                        start_front = None
                        target_front = None
                        initial_heading_at_start = None
                        cumulative_target_heading = None
                        first_turn_pending = False
                        if last_heading is not None:
                            if abs(last_heading - 0) <= 10 or abs(last_heading - 360) <= 10:
                                turn_direction = "CCW"
                                is_ccw = True
                                print(f"IMU heading {last_heading}° -> Direction: CCW")
                            elif abs(last_heading - 180) <= 10:
                                turn_direction = "CW"
                                is_ccw = False
                                print(f"IMU heading {last_heading}° -> Direction: CW")
                            else:
                                print(f"IMU heading {last_heading}° -> No valid direction determined, defaulting to CCW.")
                                turn_direction = "CCW"
                                is_ccw = True
            await asyncio.sleep(0)

    sensor_task = asyncio.create_task(sensor_drain_task())
    serial_task = asyncio.create_task(serial_drain_task())

    try:
        while True:
            now = time.time()
            if started and not parking_mode:
                if last_front is not None:
                    threshold = FIRST_FRONT_THRESHOLD if turn_count == 0 else FRONT_THRESHOLD
                    if last_front < threshold:
                        if cumulative_target_heading is None:
                            if last_heading is not None:
                                initial_heading_at_start = last_heading
                                if abs(initial_heading_at_start - 0) <= 10 or abs(initial_heading_at_start - 360) <= 10:
                                    turn_direction = "CCW"
                                    is_ccw = True
                                elif abs(initial_heading_at_start - 180) <= 10:
                                    turn_direction = "CW"
                                    is_ccw = False
                                else:
                                    turn_direction = "CCW"
                                    is_ccw = True
                                if is_ccw:
                                    cumulative_target_heading = (initial_heading_at_start - (FIRST_TURN_ANGLE + TURN_OFFSET)) % 360
                                else:
                                    cumulative_target_heading = (initial_heading_at_start + (FIRST_TURN_ANGLE + TURN_OFFSET)) % 360
                                print(f"\n(Heading sampled at turn start) initial_heading={initial_heading_at_start}°, first target={cumulative_target_heading}°")
                            else:
                                print("\nObstacle detected but no IMU heading available — performing timed turn fallback.")
                                await drive(TURN_SPEED_A, TURN_SPEED_B, SERVO_LEFT if is_ccw else SERVO_RIGHT)
                                await asyncio.sleep(1.2)
                                turn_count += 1
                                await drive(FORWARD_SPEED_A, FORWARD_SPEED_B, SERVO_FORWARD)
                                continue
                        if turn_count == 0:
                            print(f"\nObstacle! Performing FIRST turn #{turn_count+1} (angle={FIRST_TURN_ANGLE}, thr={threshold} mm). Direction={'CCW' if is_ccw else 'CW'}")
                        else:
                            print(f"\nObstacle! Performing turn #{turn_count+1} (angle={TURN_ANGLE}, thr={threshold} mm). Direction={'CCW' if is_ccw else 'CW'}")
                        turn_servo = SERVO_LEFT if is_ccw else SERVO_RIGHT
                        await drive(TURN_SPEED_A, TURN_SPEED_B, turn_servo)
                        turn_start_time = time.time()
                        ANGLE_TOLERANCE = 8.0
                        TIMEOUT = 6.0
                        while True:
                            if last_heading is not None:
                                diff = shortest_angle_diff(cumulative_target_heading, last_heading)
                                if abs(diff) <= ANGLE_TOLERANCE:
                                    break
                            if time.time() - turn_start_time > TIMEOUT:
                                print("Turn timeout, aborting turn wait.")
                                break
                            await asyncio.sleep(0.02)
                        turn_count += 1
                        if is_ccw:
                            cumulative_target_heading = (cumulative_target_heading - (TURN_ANGLE + TURN_OFFSET)) % 360
                        else:
                            cumulative_target_heading = (cumulative_target_heading + (TURN_ANGLE + TURN_OFFSET)) % 360
                        await drive(FORWARD_SPEED_A, FORWARD_SPEED_B, SERVO_FORWARD)
                        print("Resuming forward...")
                        if turn_count >= 12:
                            parking_mode = True
                            if start_front is not None:
                                target_front = start_front + PARK_OFFSET
                            else:
                                target_front = None
                            print(f"\nCompleted 12 turns. Entering parking mode! target_front={target_front} mm")
                            await brake(SERVO_FORWARD)
            elif started and parking_mode:
                if start_front is not None and last_front is not None and target_front is not None:
                    if last_front < SAFE_MIN_FRONT:
                        await brake(SERVO_FORWARD)
                        await drive(-PARK_MIN_SPEED, -PARK_MIN_SPEED, SERVO_FORWARD)
                        await asyncio.sleep(0.2)
                        await brake(SERVO_FORWARD)
                    error = target_front - last_front
                    if abs(error) <= PARK_TOLERANCE:
                        print(f"\nParking reached at {last_front:.1f} mm (target {target_front:.1f} mm). Stopping.")
                        await brake(SERVO_FORWARD)
                        started = False
                        parking_mode = False
                        initial_heading_at_start = None
                        cumulative_target_heading = None
                        continue
                    spd = int(min(PARK_MAX_SPEED, max(PARK_MIN_SPEED, abs(error) * PARK_KP)))
                    a = b = -spd if error > 0 else spd
                    if (now - last_cmd_time) >= cmd_period:
                        if (a, b) != last_drive_vals:
                            await drive(a, b, SERVO_FORWARD)
                            last_drive_vals = (a, b)
                        last_cmd_time = now
            ts = time.strftime('%H:%M:%S')
            tf = f"{target_front:.1f}" if target_front is not None else "N/A"
            lf = f"{last_front:.1f}" if last_front is not None else "N/A"
            lh = f"{last_heading:.2f}" if last_heading is not None else "N/A"
            dir_display = turn_direction if turn_direction is not None else ("CCW (default)" if is_ccw else "CW (default)")
            out = f"[{ts}] Front: {lf} mm | Heading: {lh}° | Target: {tf} mm | Turns: {turn_count} | Parking: {parking_mode} | Started: {started} | Dir: {dir_display}"
            pad = ' ' * max(0, last_len - len(out))
            sys.stdout.write('\r' + out + pad)
            sys.stdout.flush()
            last_len = len(out)
            await asyncio.sleep(0.05)
    finally:
        _stop_event.set()
        sensor_task.cancel()
        serial_task.cancel()
        t_zmq.join(timeout=0.5)
        t_serial.join(timeout=0.5)
        log("Shutting down main loop", "info")

if __name__ == "__main__":
    try:
        asyncio.run(process_queue())
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt, stopping...")
        _stop_event.set()
        time.sleep(0.2)
    except Exception as e:
        print("Fatal error:", e)
        _stop_event.set()
        raise