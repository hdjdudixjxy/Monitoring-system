import cv2
import numpy as np
import threading

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

TOPIC = "/world/crop/model/uav_with_gimbal/model/camera/link/camera_link/sensor/camera/image"   #ruta para instanciar imagen de la cámara

latest_frame = None                 
frame_lock = threading.Lock()


def image_callback(msg: Image):
    global latest_frame

    width = msg.width
    height = msg.height
    
    raw = msg.data

    if width == 0 or height == 0 or len(raw) == 0:
        return

    expected_size = width * height * 3
    if len(raw) < expected_size:
        return

    img = np.frombuffer(raw[:expected_size], dtype=np.uint8)
    img = img.reshape((height, width, 3))

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    with frame_lock:
        latest_frame = img

def main():
    global latest_frame

    node = Node()

    ok = node.subscribe(Image, TOPIC, image_callback)
    if not ok:
        print(f"No se pudo suscribir al topic: {TOPIC}")
        return

    while True:
        frame_to_show = None

        with frame_lock:
            if latest_frame is not None:
                frame_to_show = latest_frame.copy()

        if frame_to_show is not None:
            cv2.imshow("Gazebo Camera (Transport Subscriber)", frame_to_show)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()