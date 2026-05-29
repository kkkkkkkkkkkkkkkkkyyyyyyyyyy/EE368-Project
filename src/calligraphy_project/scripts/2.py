import cv2

cap = cv2.VideoCapture(2) # 还是用你之前成功的那个索引

print("正在显示原始画面，按 'q' 退出...")
while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取到画面！")
        break
    
    # 强制缩小显示尺寸，防止窗口过大显示不出
    resized_frame = cv2.resize(frame, (640, 480))
    cv2.imshow("Original Camera Feed", resized_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()