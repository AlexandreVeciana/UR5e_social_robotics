from ultralytics import YOLO
import cv2
import time
import sys
import numpy as np

MODEL_PATH = "yolo11n-seg.pt"
CAMERA_INDEX = 0
IMG_SIZE = 640
CONF_THRESHOLD = 0.25
WINDOW_NAME = "YOLO Person Segmentation"

PERSON_CLASS_NAME = "person"

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

        display_frame = frame.copy()
        person_count = 0

        if (
            result.boxes is not None
            and result.masks is not None
            and result.masks.data is not None
        ):
            boxes = result.boxes
            masks = result.masks.data.cpu().numpy()

            for i, box in enumerate(boxes):
                class_id = int(box.cls[0])
                class_name = result.names[class_id]

                if class_name != PERSON_CLASS_NAME:
                    continue

                person_count += 1
                confidence = float(box.conf[0])

                # Mask is resized to original frame size
                mask = masks[i]
                mask = cv2.resize(
                    mask,
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_NEAREST
                )

                binary_mask = (mask > 0.5).astype(np.uint8)

                # Compute centroid from segmentation mask
                moments = cv2.moments(binary_mask)

                if moments["m00"] > 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                else:
                    continue

                # Draw mask overlay
                colored_mask = np.zeros_like(display_frame)
                colored_mask[binary_mask == 1] = (0, 255, 0)

                display_frame = cv2.addWeighted(
                    display_frame,
                    1.0,
                    colored_mask,
                    0.35,
                    0
                )

                # Draw contour
                contours, _ = cv2.findContours(
                    binary_mask,
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE
                )

                cv2.drawContours(
                    display_frame,
                    contours,
                    -1,
                    (0, 255, 0),
                    2
                )

                # Draw centroid
                cv2.circle(
                    display_frame,
                    (cx, cy),
                    6,
                    (0, 0, 255),
                    -1
                )

                # Draw label
                label = f"person {confidence:.2f} centroid=({cx},{cy})"

                cv2.putText(
                    display_frame,
                    label,
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )

        info_text = f"persons={person_count} | {elapsed_ms:.1f} ms"

        cv2.rectangle(
            display_frame,
            (0, 0),
            (display_frame.shape[1], 40),
            (0, 0, 0),
            -1
        )

        cv2.putText(
            display_frame,
            info_text,
            (15, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.imshow(WINDOW_NAME, display_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Exit requested by user")
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    print("Camera released correctly")