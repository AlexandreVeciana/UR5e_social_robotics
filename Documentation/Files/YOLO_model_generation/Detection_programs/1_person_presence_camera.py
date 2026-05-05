from ultralytics import YOLO
import cv2
import time
import sys

MODEL_PATH = "yolo11n-cls.pt"
CAMERA_INDEX = 0
IMG_SIZE = 224
WINDOW_NAME = "YOLO Person Presence (Classification)"

TARGET_CLASS = "person"

model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Error: camera could not be opened")
    sys.exit(1)

print("Camera opened successfully")
print("Press 'q' to quit")

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
            verbose=False
        )

        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        result = results[0]

        if result.probs is not None:

            class_id = int(result.probs.top1)
            confidence = float(result.probs.top1conf)
            class_name = result.names[class_id]

            if class_name == TARGET_CLASS:
                text = f"PERSON DETECTED ({confidence:.2f}) - {elapsed_ms:.1f} ms"
                color = (0,255,0)
            else:
                text = f"NO PERSON ({class_name}) - {elapsed_ms:.1f} ms"
                color = (0,0,255)

        else:
            text = f"NO CLASSIFICATION - {elapsed_ms:.1f} ms"
            color = (0,0,255)

        cv2.rectangle(frame,(0,0),(frame.shape[1],55),(0,0,0),-1)

        cv2.putText(frame,text,(20,38),
                    cv2.FONT_HERSHEY_SIMPLEX,1.0,color,2,cv2.LINE_AA)

        cv2.imshow(WINDOW_NAME,frame)

        if cv2.waitKey(1)&0xFF==ord("q"):
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    print("Camera released correctly")