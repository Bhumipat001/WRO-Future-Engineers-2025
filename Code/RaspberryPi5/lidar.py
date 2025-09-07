import asyncio
import sys
import time
from rplidarc1 import RPLidar

PORT = "/dev/lidar"
BAUDRATE = 460800
ANGLE_WIDTH = 10
POINTS_PER_SIDE = 1
FRONT_ANGLE_WIDTH = 1
ZMQ_PORT = 5000

PUBLISH_INTERVAL = 0.02
ZMQ_SNDHWM = 1

lidar = RPLidar(PORT, BAUDRATE)


def _angle_diff(a, b):
    diff = (a - b + 180.0) % 360.0 - 180.0
    return abs(diff)


def filter_points(points, target_angle, width=ANGLE_WIDTH, count=POINTS_PER_SIDE):
    filtered = [p['d_mm'] for p in points
                if p.get('d_mm') is not None and _angle_diff(p.get('a_deg', 0.0), target_angle) <= width]
    filtered.sort()
    return filtered[:count]


async def process_scan_data():
    zmq_ctx = None
    zmq_sock = None
    try:
        import zmq
        zmq_ctx = zmq.Context()
        zmq_sock = zmq_ctx.socket(zmq.PUB)
        try:
            zmq_sock.setsockopt(zmq.SNDHWM, ZMQ_SNDHWM)
        except Exception:
            pass
        try:
            zmq_sock.setsockopt(zmq.LINGER, 0)
        except Exception:
            pass
        bind_addr = f"tcp://*:{ZMQ_PORT}"
        zmq_sock.bind(bind_addr)
        print(f"ZMQ publisher bound to {bind_addr}")
    except Exception as e:
        print("ZMQ not available or bind failed, publishing disabled:", e)
        zmq_sock = None

    try:
        async with asyncio.TaskGroup() as tg:
            tg.create_task(process_queue(lidar.output_queue, lidar.stop_event, zmq_sock))
            tg.create_task(lidar.simple_scan(make_return_dict=True))
    finally:
        try:
            if zmq_sock is not None:
                zmq_sock.close()
            if zmq_ctx is not None:
                zmq_ctx.term()
        except Exception:
            pass


async def process_queue(queue, stop_event, zmq_socket=None):
    last_front = None
    last_left = None
    last_right = None
    last_len = 0

    seq = 0
    while True:
        all_points = []
        try:
            while True:
                data = queue.get_nowait()
                if asyncio.iscoroutine(data):
                    data = await data
                all_points.append(data)
        except asyncio.QueueEmpty:
            pass

        if all_points:
            front_point = filter_points(all_points, 0, width=FRONT_ANGLE_WIDTH)
            left_point = filter_points(all_points, 90)
            right_point = filter_points(all_points, 270)

            last_front = front_point[0] if front_point else last_front
            last_left = left_point[0] if left_point else last_left
            last_right = right_point[0] if right_point else last_right

        ts = time.strftime('%H:%M:%S')
        out = (f"[{ts}] Front: {last_front if last_front is not None else 'N/A'} mm | "
               f"Left: {last_left if last_left is not None else 'N/A'} mm | "
               f"Right: {last_right if last_right is not None else 'N/A'} mm")

        pad = ' ' * max(0, last_len - len(out))
        sys.stdout.write('\r' + out + pad)
        sys.stdout.flush()
        last_len = len(out)

        if zmq_socket is not None:
            seq += 1
            payload = {
                'seq': seq,
                'ts': ts,
                'front_mm': last_front,
                'left_mm': last_left,
                'right_mm': last_right,
            }
            try:
                zmq_socket.send_json(payload, flags=getattr(__import__('zmq'), 'NOBLOCK', 0))
            except Exception:
                pass

        await asyncio.sleep(PUBLISH_INTERVAL)


try:
    asyncio.run(process_scan_data())
except KeyboardInterrupt:
    print("Stopping Lidar...")
    try:
        lidar.reset()
    except Exception:
        pass