import cv2

DEVICE = "/dev/video0"

print("Dang mo Astra Pro...")

camera = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

if not camera.isOpened():
    print("KHONG MO DUOC CAMERA")
    raise SystemExit


# =========================
# CAU HINH CAMERA
# =========================

# Bat buoc dung MJPG
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
camera.set(cv2.CAP_PROP_FOURCC, fourcc)

# Do phan giai
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Tam thoi dung 15 FPS cho on dinh tren WSL2
camera.set(cv2.CAP_PROP_FPS, 15)


# =========================
# DOC LAI THONG SO
# =========================

width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = camera.get(cv2.CAP_PROP_FPS)

fourcc_value = int(camera.get(cv2.CAP_PROP_FOURCC))

fourcc_text = "".join([
    chr((fourcc_value >> 0) & 0xFF),
    chr((fourcc_value >> 8) & 0xFF),
    chr((fourcc_value >> 16) & 0xFF),
    chr((fourcc_value >> 24) & 0xFF)
])


print("DA MO DUOC CAMERA")
print("Resolution:", width, "x", height)
print("Format:", fourcc_text)
print("FPS:", fps)
print("Nhan Q de thoat")


# =========================
# DOC HINH
# =========================

while True:

    ret, frame = camera.read()

    if not ret:
        print("KHONG DOC DUOC FRAME")
        break

    cv2.imshow("Astra Pro RGB", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break


camera.release()
cv2.destroyAllWindows()