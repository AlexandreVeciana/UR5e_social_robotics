from ultralytics import YOLO
import cv2
import time
import sys

MODEL_PATH = "yolo11n.pt"
CAMERA_INDEX = 0
IMG_SIZE = 640
CONF_THRESHOLD = 0.25
WINDOW_NAME = "YOLO Person Detection"

model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Error: camera could not be opened")
    sys.exit(1)

print("Camera opened successfully")
print("Press 'q' to quit")

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: could not read frame")
            break

        start_time = time.perf_counter()

        results = model.predict(
            source=frame,
            imgsz=IMG_SIZE,
            conf=CONF_THRESHOLD,
            verbose=False
        )

        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        result = results[0]

        detections = 0

        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = result.names[class_id]

                if class_name != "person":
                    continue

                detections += 1

                confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                label = f"person {confidence:.2f} center=({cx},{cy})"

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                cv2.putText(frame, label, (x1, max(y1 - 10, 25)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2, cv2.LINE_AA)

        info = f"persons={detections} | {elapsed_ms:.1f} ms"
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 35), (0, 0, 0), -1)
        cv2.putText(frame, info, (15, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(WINDOW_NAME, frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    print("Camera released correctly")