import cv2
import numpy as np
import threading

from ultralytics import YOLO
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

TOPIC = "/world/crop/model/uav_with_gimbal/model/camera/link/camera_link/sensor/camera/image"
MODEL_PATH = "best.pt"

latest_frame = None
frame_lock = threading.Lock()

# Cargar modelo YOLO
model = YOLO(MODEL_PATH)

CLASS_COLORS = {
    "Red Rot": (0, 0, 255),      # rojo
    "Yellow": (0, 255, 255),     # amarillo
    "Rust": (0, 165, 255),       # naranja
    "Mosaic": (255, 255, 0)      # celeste
}

DEFAULT_COLOR = (255, 255, 255)

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

    # Gazebo entrega RGB, OpenCV usa BGR
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    with frame_lock:
        latest_frame = img


def draw_segmentation_polygons(frame, results):

    annotated = frame.copy()

    for r in results:
        names = r.names

        if r.masks is None:
            continue

        boxes = r.boxes
        polygons = r.masks.xy

        for i, poly in enumerate(polygons):

            poly = np.array(poly, dtype=np.int32).reshape((-1, 1, 2))

            cls_name = "unknown"
            conf = 0.0

            if boxes is not None and i < len(boxes):

                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())

                if cls_id in names:
                    cls_name = names[cls_id]

            color = CLASS_COLORS.get(cls_name, DEFAULT_COLOR)

            cv2.polylines(
                annotated,
                [poly],
                True,
                color,
                2
            )

            x0, y0 = poly[0][0]

            label = f"{cls_name} {conf:.2f}"

            cv2.putText(
                annotated,
                label,
                (int(x0), int(y0) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA
            )

    return annotated


def main():
    global latest_frame

    node = Node()

    ok = node.subscribe(Image, TOPIC, image_callback)
    if not ok:
        print(f"No se pudo suscribir al topic: {TOPIC}")
        return

    print(f"Suscrito a: {TOPIC}")
    print("Modelo cargado:", MODEL_PATH)
    print("Presiona ESC para salir.")

    while True:
        frame_to_process = None

        with frame_lock:
            if latest_frame is not None:
                frame_to_process = latest_frame.copy()

        if frame_to_process is not None:
            # Inferencia YOLO
            results = model.predict(
                source=frame_to_process,
                imgsz=640,
                conf=0.62,
                verbose=False
            )

            # Dibujar solo polígonos
            annotated = draw_segmentation_polygons(frame_to_process, results)

            cv2.imshow("Gazebo Camera (Transport Subscriber) + YOLO Segmentation", annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()