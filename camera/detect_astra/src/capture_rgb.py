import cv2
from pathlib import Path


# =========================
# CAU HINH
# =========================

DEVICE = "/dev/video0"
WIDTH = 1280
HEIGHT = 720
FPS = 30

# =========================
# THU MUC LUU ANH
# =========================

PROJECT_DIR = Path(__file__).resolve().parent.parent
SAVE_DIR = PROJECT_DIR / "data" / "rgb"

SAVE_DIR.mkdir(parents=True, exist_ok=True)


# =========================
# MO CAMERA
# =========================

camera = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

if not camera.isOpened():
    print("KHONG MO DUOC CAMERA")
    raise SystemExit


# =========================
# CAU HINH CAMERA
# =========================

fourcc = cv2.VideoWriter_fourcc(*"MJPG")

camera.set(cv2.CAP_PROP_FOURCC, fourcc)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
camera.set(cv2.CAP_PROP_FPS, FPS)

# Giam frame cu ton trong buffer
camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)


# =========================
# DOC LAI THONG SO CAMERA
# =========================

actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
actual_fps = camera.get(cv2.CAP_PROP_FPS)

print("DA MO ASTRA PRO")
print("Resolution:", actual_width, "x", actual_height)
print("FPS:", actual_fps)
print("S = chup anh")
print("Q = thoat")


# =========================
# DEM ANH CU
# =========================

existing_images = list(SAVE_DIR.glob("pallet_*.jpg"))
count = len(existing_images)

print("So anh hien tai:", count)


# =========================
# CAMERA LOOP
# =========================

while True:

    ret, frame = camera.read()

    if not ret:
        print("KHONG DOC DUOC FRAME")
        break


    # Tao ban sao CHI de hien thi
    # Anh goc "frame" se duoc dung de luu
    display_frame = frame.copy()

    text = f"Images: {count}"

    cv2.putText(
        display_frame,
        text,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        2
    )


    # Hien thi camera
    cv2.imshow(
        "Astra Pro - Dataset Capture",
        display_frame
    )


    key = cv2.waitKey(1) & 0xFF


    # =========================
    # S = SAVE
    # =========================

    if key == ord("s"):

        filename = SAVE_DIR / f"pallet_{count:04d}.jpg"

        # Luu FRAME GOC
        # Khong co chu Images tren anh dataset
        cv2.imwrite(
            str(filename),
            frame
        )

        print("DA LUU:", filename)

        count += 1


    # =========================
    # Q = QUIT
    # =========================

    elif key == ord("q"):

        break


# =========================
# DONG CAMERA
# =========================

camera.release()
cv2.destroyAllWindows()

print("DA DONG CAMERA")