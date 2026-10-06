import cv2
import os
import time
import torch
from pathlib import Path
from ultralytics import YOLO


# ============================================================
# 1. CẤU HÌNH
# ============================================================

DEVICE = os.environ.get("PALLET_CAMERA", "/dev/video0")

WIDTH = 640
HEIGHT = 480
FPS = 30

# Confidence nhận diện
CONF = 0.70

# Kích thước ảnh đưa vào YOLO
IMGSZ = 640


# ============================================================
# 2. ĐƯỜNG DẪN PROJECT VÀ MODEL
# ============================================================

# File hiện tại:
# astra_pallet/src/test_pallets.py
#
# parent       = src
# parent.parent = astra_pallet

PROJECT_DIR = Path(__file__).resolve().parent.parent

MODEL_PATH = PROJECT_DIR / "models" / "best_v2.pt"


print("=" * 60)
print("PROJECT :", PROJECT_DIR)
print("MODEL   :", MODEL_PATH)
print("EXISTS  :", MODEL_PATH.exists())
print("=" * 60)


# Kiểm tra model có tồn tại hay không
if not MODEL_PATH.exists():
    print(f"❌ KHÔNG TÌM THẤY {MODEL_PATH.name}")
    print("Model phải nằm tại:")
    print(MODEL_PATH)
    raise SystemExit(1)


# ============================================================
# 3. LOAD YOLO
# ============================================================

print("\nĐang load YOLO model...")

model = YOLO(str(MODEL_PATH))
INFERENCE_DEVICE = 0 if torch.cuda.is_available() else "cpu"
print("Inference device:", INFERENCE_DEVICE)

print("✅ Load model thành công!")
print("Classes:", model.names)


# ============================================================
# 4. MỞ CAMERA ASTRA PRO
# ============================================================

print("\nĐang mở camera:", DEVICE)

camera = cv2.VideoCapture(
    DEVICE,
    cv2.CAP_V4L2
)


# Astra Pro RGB dùng MJPG
fourcc = cv2.VideoWriter_fourcc(*"MJPG")

camera.set(
    cv2.CAP_PROP_FOURCC,
    fourcc
)

camera.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    WIDTH
)

camera.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    HEIGHT
)

camera.set(
    cv2.CAP_PROP_FPS,
    FPS
)

# Giảm độ trễ do buffer
camera.set(
    cv2.CAP_PROP_BUFFERSIZE,
    1
)


# ============================================================
# 5. KIỂM TRA CAMERA
# ============================================================

if not camera.isOpened():

    print("❌ Không mở được camera:", DEVICE)

    print("\nHãy kiểm tra bằng:")
    print("ls /dev/video*")

    raise SystemExit(1)


actual_width = int(
    camera.get(cv2.CAP_PROP_FRAME_WIDTH)
)

actual_height = int(
    camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
)

actual_fps = camera.get(
    cv2.CAP_PROP_FPS
)


print("✅ Camera đã mở thành công!")

print(
    f"Camera resolution: "
    f"{actual_width}x{actual_height}"
)

print(
    f"Camera FPS: "
    f"{actual_fps:.1f}"
)

print("\nĐưa pallet trước camera.")
print("Nhấn Q để thoát.")


# ============================================================
# 6. VÒNG LẶP NHẬN DIỆN
# ============================================================

prev_time = time.time()


while True:

    # ----------------------------------
    # Đọc frame camera
    # ----------------------------------

    ret, frame = camera.read()


    if not ret:

        print(
            "❌ Không đọc được frame từ camera."
        )

        break


    # ----------------------------------
    # YOLO SEGMENTATION
    # ----------------------------------

    results = model.predict(
        source=frame,
        conf=CONF,
        imgsz=IMGSZ,
        device=INFERENCE_DEVICE,
        verbose=False
    )


    result = results[0]


    # ----------------------------------
    # Vẽ segmentation mask + box
    # ----------------------------------

    annotated_frame = result.plot()


    # ----------------------------------
    # Tính FPS thực tế
    # ----------------------------------

    current_time = time.time()

    elapsed = current_time - prev_time


    if elapsed > 0:

        realtime_fps = 1.0 / elapsed

    else:

        realtime_fps = 0


    prev_time = current_time


    # ----------------------------------
    # Đếm số object phát hiện
    # ----------------------------------

    detected_objects = 0


    if result.boxes is not None:

        detected_objects = len(
            result.boxes
        )


    # ----------------------------------
    # Hiển thị FPS
    # ----------------------------------

    cv2.putText(
        annotated_frame,
        f"FPS: {realtime_fps:.1f}",
        (20, 35),

        cv2.FONT_HERSHEY_SIMPLEX,

        0.8,

        (0, 255, 0),

        2
    )


    # ----------------------------------
    # Hiển thị số pallet phát hiện
    # ----------------------------------

    cv2.putText(
        annotated_frame,
        f"Detected: {detected_objects}",
        (20, 70),

        cv2.FONT_HERSHEY_SIMPLEX,

        0.8,

        (0, 255, 0),

        2
    )


    # ----------------------------------
    # Hiển thị camera
    # ----------------------------------

    cv2.imshow(
        "Astra Pro - Pallet YOLO11 Segmentation",
        annotated_frame
    )


    # ----------------------------------
    # Q = thoát
    # ----------------------------------

    key = cv2.waitKey(1) & 0xFF


    if key == ord("q"):

        print("\nĐang thoát...")

        break


# ============================================================
# 7. GIẢI PHÓNG CAMERA
# ============================================================

camera.release()

cv2.destroyAllWindows()

print("✅ Đã đóng camera.")
