import rospy
from geometry_msgs.msg import Point  # 使用 ROS 的点坐标消息
from ultralytics import YOLO
import cv2

# 初始化 ROS 节点
rospy.init_node('ink_detector_node', anonymous=True)
pub = rospy.Publisher('/ink_tray_position', Point, queue_size=10)

model = YOLO('best.pt')
cap = cv2.VideoCapture(1) # 记得换成你测试成功的那个索引

while not rospy.is_shutdown():
    success, frame = cap.read()
    if success:
        results = model(frame)
        for r in results:
            boxes = r.boxes
            if len(boxes) > 0:
                x1, y1, x2, y2 = boxes[0].xyxy[0].cpu().numpy()
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                
                # 发布坐标到 ROS
                msg = Point()
                msg.x, msg.y = float(cx), float(cy)
                pub.publish(msg)
                
                # 顺便在屏幕显示
                cv2.imshow("YOLO to ROS", results[0].plot())
                if cv2.waitKey(1) == ord('q'): break