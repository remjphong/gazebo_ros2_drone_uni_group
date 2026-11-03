import time

from gz.msgs.image_pb2 import Image
from gz.transport import Node

TOPIC = (
    "/world/obstacle_course/model/x500_lidar_rgb_0/"
    "link/camera_link/sensor/IMX214/image"
)

received = False


def on_image(msg):
    global received

    if received:
        return

    received = True

    print(f"Image: {msg.width} x {msg.height}")
    print(f"Pixel format: {msg.pixel_format_type}")
    print(f"Data bytes: {len(msg.data)}")


node = Node()

if not node.subscribe(Image, TOPIC, on_image):
    print("Could not subscribe to camera topic")
    raise SystemExit(1)

print("Waiting for a camera frame... Ctrl+C to stop")

try:
    while not received:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
