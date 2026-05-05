from ultralytics import YOLO
import cv2
import time
import sys
import csv
import os

# ============================================================
# PARAMETERS
# ============================================================

MODEL_PATH = "yolo11n-pose.pt"      # Important: model trained for mouse pose
VIDEO_PATH = "Documentation/Files/YOLO_model_generation/Detection_programs/mouse_1.mp4"

IMG_SIZE = 640
CONF_THRESHOLD = 0.25
KEYPOINT_CONF_THRESHOLD = 0.25

WINDOW_NAME = "Mouse Pose Tracking"

SAVE_VIDEO = True
OUTPUT_VIDEO = "mouse_pose_tracking_output.mp4"
OUTPUT_CSV = "mouse_keypoints_tracking.csv"

# Example keypoint names.
# Change these names according to your trained mouse pose model.
KEYPOINT_NAMES = [
    "nose",
    "left_ear",
    "right_ear",
    "neck",
    "spine",
    "tail_base",
    "tail_mid",
    "tail_tip",
    "left_front_paw",
    "right_front_paw",
    "left_hind_paw",
    "right_hind_paw"
]

# ============================================================
# LOAD MODEL
# ============================================================

model = YOLO(MODEL_PATH)

# ============================================================
# OPEN VIDEO
# ============================================================

cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    print(f"Error: video could not be opened: {VIDEO_PATH}")
    sys.exit(1)

fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

print("Video opened successfully")
print(f"Resolution: {width}x{height}")
print(f"FPS: {fps}")
print(f"Total frames: {total_frames}")
print("Press 'q' to quit")

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

# ============================================================
# VIDEO WRITER
# ============================================================

writer = None

if SAVE_VIDEO:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (width, height))

# ============================================================
# CSV FILE
# ============================================================

csv_file = open(OUTPUT_CSV, mode="w", newline="")
csv_writer = csv.writer(csv_file)

header = ["frame", "time_sec", "track_id"]

for name in KEYPOINT_NAMES:
    header += [f"{name}_x", f"{name}_y", f"{name}_conf"]

csv_writer.writerow(header)

# ============================================================
# MAIN LOOP
# ============================================================

frame_idx = 0

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("End of video")
            break

        start_time = time.perf_counter()

        # Use track instead of predict to maintain object identity between frames
        results = model.track(
            source=frame,
            imgsz=IMG_SIZE,
            conf=CONF_THRESHOLD,
            persist=True,
            verbose=False
        )

        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        result = results[0]

        annotated_frame = result.plot()

        time_sec = frame_idx / fps if fps > 0 else 0.0

        # ------------------------------------------------------------
        # Read tracking IDs
        # ------------------------------------------------------------
        track_ids = []

        if result.boxes is not None and result.boxes.id is not None:
            track_ids = result.boxes.id.cpu().numpy().astype(int).tolist()

        # ------------------------------------------------------------
        # Read keypoints
        # ------------------------------------------------------------
        if result.keypoints is not None and result.keypoints.xy is not None:
            kpts_all = result.keypoints.xy.cpu().numpy()

            if result.keypoints.conf is not None:
                confs_all = result.keypoints.conf.cpu().numpy()
            else:
                confs_all = None

            for obj_idx, kpts in enumerate(kpts_all):

                if obj_idx < len(track_ids):
                    track_id = track_ids[obj_idx]
                else:
                    track_id = obj_idx

                row = [frame_idx, f"{time_sec:.3f}", track_id]

                for kp_idx, name in enumerate(KEYPOINT_NAMES):

                    if kp_idx < len(kpts):
                        x, y = kpts[kp_idx]

                        if confs_all is not None:
                            conf = confs_all[obj_idx][kp_idx]
                        else:
                            conf = 1.0

                        if conf >= KEYPOINT_CONF_THRESHOLD:
                            row += [f"{x:.2f}", f"{y:.2f}", f"{conf:.3f}"]
                        else:
                            row += ["", "", f"{conf:.3f}"]
                    else:
                        row += ["", "", ""]

                csv_writer.writerow(row)

        # ------------------------------------------------------------
        # Overlay information
        # ------------------------------------------------------------
        info_lines = [
            f"frame: {frame_idx}/{total_frames}",
            f"time: {time_sec:.2f} s",
            f"inference: {elapsed_ms:.1f} ms",
            f"model: {os.path.basename(MODEL_PATH)}"
        ]

        cv2.rectangle(annotated_frame, (0, 0), (520, 130), (0, 0, 0), -1)

        for i, line in enumerate(info_lines):
            cv2.putText(
                annotated_frame,
                line,
                (15, 30 + i * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

        # ------------------------------------------------------------
        # Show and save
        # ------------------------------------------------------------
        cv2.imshow(WINDOW_NAME, annotated_frame)

        if writer is not None:
            writer.write(annotated_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print("Stopped by user")
            break

        frame_idx += 1

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    cap.release()

    if writer is not None:
        writer.release()

    csv_file.close()

    cv2.destroyAllWindows()
    cv2.waitKey(1)

    print("Video released correctly")
    print(f"CSV saved as: {OUTPUT_CSV}")

    if SAVE_VIDEO:
        print(f"Annotated video saved as: {OUTPUT_VIDEO}")