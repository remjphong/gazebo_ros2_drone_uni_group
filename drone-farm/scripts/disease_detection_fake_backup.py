import cv2
import numpy as np
import time

print("Đang chạy chế độ GIẢ LẬP camera để test thuật toán...")
print("Nhấn 'q' trên cửa sổ video để thoát.")

class FakeCamera:
    def __init__(self):
        self.t = 0
    def isOpened(self): return True
    def read(self):
        # Tạo khung hình nền xanh (cánh đồng)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (50, 150, 50) # Màu xanh lá (BGR)
        
        # Di chuyển đốm bệnh qua lại để giống drone đang bay quét
        x = int(320 + 200 * np.sin(self.t))
        y = int(240 + 100 * np.cos(self.t * 0.5))
        
        # Vẽ một đốm màu đỏ cam (bệnh nặng) và một đốm vàng (bệnh nhẹ)
        cv2.circle(frame, (x, y), 40, (30, 80, 220), -1) # Đỏ cam (BGR)
        cv2.circle(frame, (x - 150, y + 100), 30, (40, 200, 250), -1) # Vàng (BGR)
        
        self.t += 0.05
        time.sleep(0.03)
        return True, frame
    def release(self): pass

cap = FakeCamera()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Chuyển sang HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Dải màu VÀNG (Bệnh nhẹ)
    lower_yellow = np.array([15, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    
    # Dải màu ĐỎ CAM (Bệnh nặng)
    lower_red_orange = np.array([0, 100, 100])
    upper_red_orange = np.array([15, 255, 255])

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_red = cv2.inRange(hsv, lower_red_orange, upper_red_orange)
    mask = cv2.bitwise_or(mask_yellow, mask_red)
    
    disease_ratio = np.count_nonzero(mask) / mask.size
    
    if disease_ratio > 0.005:
        print(f"⚠️ PHÁT HIỆN SÂU BỆNH! Tỷ lệ diện tích: {disease_ratio:.2%}")
        cv2.putText(frame, "CANH BAO: PHAT HIEN SAU BENH!", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Drone Camera - Live Feed", frame)
    cv2.imshow("Disease Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
