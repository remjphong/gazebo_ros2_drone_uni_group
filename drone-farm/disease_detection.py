import cv2
import numpy as np

# 1. Cấu hình GStreamer pipeline để nhận luồng H.264 từ Gazebo (UDP port 5600)
gst_pipeline = (
    "udpsrc port=5600 "
    "caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! "
    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
)

# Mở luồng video bằng GStreamer backend của OpenCV
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("LỖI: Không thể mở luồng video qua GStreamer.")
    print("Hãy kiểm tra:")
    print("1. Terminal 1 đã chạy PX4 với gz_x500_mono_cam chưa?")
    print("2. Đã cài đủ các gói GStreamer (gstreamer1.0-plugins-good, bad, ugly, libav) chưa?")
    exit()

print("✅ Đã kết nối camera thành công!")
print("Đang chờ drone cất cánh... Nhấn 'q' trên cửa sổ video để thoát.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Không nhận được frame hình ảnh. Kiểm tra lại kết nối Gazebo...")
        break

    # 2. Xử lý ảnh: Chuyển sang hệ màu HSV để nhận diện màu nâu
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Dải màu nâu (giả lập sâu bệnh)
    lower_brown = np.array([10, 50, 50])
    upper_brown = np.array([30, 255, 255])
    
    # Tạo mặt nạ (Mask)
    mask = cv2.inRange(hsv, lower_brown, upper_brown)
    
    # Tính tỷ lệ diện tích vùng bệnh
    disease_ratio = np.count_nonzero(mask) / mask.size
    
    if disease_ratio > 0.005: # Ngưỡng phát hiện 0.5%
        print(f"⚠️ PHÁT HIỆN SÂU BỆNH! Tỷ lệ diện tích: {disease_ratio:.2%}")
        cv2.putText(frame, "CANH BAO: SAU BENH!", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # 3. Hiển thị kết quả
    cv2.imshow("Drone Camera - Disease Detection", frame)
    cv2.imshow("Mask", mask)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
