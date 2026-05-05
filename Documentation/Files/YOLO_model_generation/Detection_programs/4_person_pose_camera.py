from ultralytics import YOLO
import cv2
import time
import sys

MODEL_PATH = "yolo11n-pose.pt"
CAMERA_INDEX = 0
IMG_SIZE = 640
CONF_THRESHOLD = 0.35
KEYPOINT_CONF_THRESHOLD = 0.35
WINDOW_NAME = "YOLO Pose - Head and Hands"

NOSE = 0
LEFT_WRIST = 9
RIGHT_WRIST = 10

model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Error: camera could not be opened")
    sys.exit(1)

print("Camera opened successfully")
print("Press 'q' to quit")

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)


def format_point(label, kpts, confs, idx):
    x, y = kpts[idx]
    conf = confs[idx]

    if conf < KEYPOINT_CONF_THRESHOLD:
        return f"{label}: not visible"

    return f"{label}: x={x:.0f}, y={y:.0f}, c={conf:.2f}"


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

        annotated_frame = result.plot()

        lines = [
            "head: not visible",
            "left_wrist: not visible",
            "right_wrist: not visible",
            f"time: {elapsed_ms:.1f} ms"
        ]

        if result.keypoints is not None and result.keypoints.xy is not None:
            kpts_all = result.keypoints.xy.cpu().numpy()
            confs_all = result.keypoints.conf.cpu().numpy()

            if len(kpts_all) > 0:
                kpts = kpts_all[0]
                confs = confs_all[0]

                lines = [
                    format_point("head", kpts, confs, NOSE),
                    format_point("left_wrist", kpts, confs, LEFT_WRIST),
                    format_point("right_wrist", kpts, confs, RIGHT_WRIST),
                    f"time: {elapsed_ms:.1f} ms"
                ]

        x0 = 15
        y0 = 30

        cv2.rectangle(annotated_frame, (0, 0), (480, 150), (0, 0, 0), -1)

        for i, line in enumerate(lines):
            cv2.putText(annotated_frame, line, (x0, y0 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(WINDOW_NAME, annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    print("Camera released correctly")