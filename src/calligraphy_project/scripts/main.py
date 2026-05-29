#!/usr/bin/env python3
import rospy
from ultralytics import YOLO
import cv2
import copy
from test2 import KinovaCalligraphy 

class RobotManager:
    def __init__(self):
        self.robot = KinovaCalligraphy()
        self.SCALE_X = 0.0005
        self.SCALE_Y = 0.0005

        # 2. 【核心补偿值】：微调这个数值！
        # 如果机械臂往左偏了，增加 OFFSET_Y；如果往右偏了，减小 OFFSET_Y
        self.OFFSET_X = 0.12 # 单位：米（5毫米）
        self.OFFSET_Y = 0.03 # 单位：米（5毫米）
        
    def run_process(self):
        model = YOLO('best.pt')
        cap = cv2.VideoCapture(2)
        
        rospy.loginfo("系统已启动，实时窗口已弹出...")
        
        # ====================================================
        # 🚨 1. 【新增】：先让机械臂抬头，进入鹰眼巡视模式
        # ====================================================
        self.robot.move_to_scan_position()
        rospy.sleep(1.5) # 等待机械臂停稳，防止画面模糊导致误识别
        
        while not rospy.is_shutdown():
            success, frame = cap.read()
            if not success: continue
            
            # 1. 图像格式处理
            if len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            else:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 2. 推理并获取结果
            results = model(frame, imgsz=640, conf=0.25)
            
        
            # 3. 【实时显示窗口】
            annotated_frame = results[0].plot()
            
            # ====== 【新增】：画出绝对中心十字准星 ======
            # 画红色竖线和横线 (假设分辨率 640x480，中心就是 320, 240)
            cv2.line(annotated_frame, (320, 0), (320, 480), (255, 0, 0), 1) 
            cv2.line(annotated_frame, (0, 240), (640, 240), (255, 0, 0), 1)
            # 画一个绿色靶心点
            cv2.circle(annotated_frame, (320, 240), 4, (0, 255, 0), -1)
            # ============================================
            
            show_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("Robot Vision: Ink Detection", show_frame)
            cv2.waitKey(1)

            # 4. 【新增】：如果视野里没有墨盘，给个提示并继续找
            if len(results[0].boxes) == 0:
                rospy.loginfo_throttle(2.0, "👀 鹰眼模式下未发现墨水盘，持续监控中...")
                continue

            # 5. 结果处理
            for r in results:
                if len(r.boxes) > 0:
                    x1, y1, x2, y2 = r.boxes[0].xyxy[0].cpu().numpy()
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    
                    rospy.loginfo(f"锁定墨盘中心！像素坐标: X={cx:.1f}, Y={cy:.1f}")
                    
                    # ====================================================
                    # 🚨 2. 【核心修正：眼在手的相对坐标计算】
                    # ====================================================
                    # 假设你的相机画面分辨率是 640x480
                    CENTER_X = 320.0
                    CENTER_Y = 240.0
                    
                    # 计算目标偏离画面正中心的像素差值 (dx, dy 有正有负)
                    dx = cx - CENTER_X
                    dy = cy - CENTER_Y
                    
                    # 算出实际给机械臂的【相对移动量】
                    real_x = (dx * self.SCALE_X) + self.OFFSET_X
                    real_y = (dy * self.SCALE_Y) + self.OFFSET_Y
                    
                    # ====================================================
                    # 【核心查错代码】：让程序自己坦白它算出了什么！
                    rospy.loginfo(f"==== 眼在手 终极调试信息 ====")
                    rospy.loginfo(f"偏离中心像素: dx={dx:.1f}, dy={dy:.1f}")
                    rospy.loginfo(f"你设置的 OFFSET_X: {self.OFFSET_X}, OFFSET_Y: {self.OFFSET_Y}")
                    rospy.loginfo(f"最终传给机械臂的相对移动量: real_x={real_x:.4f}, real_y={real_y:.4f}")
                    # ====================================================
                    
                    # # 第一步：去蘸水 (此时机械臂是在鹰眼位置的基础上，偏移 real_x, real_y)
                    # self.robot.dip_ink_action(real_x, real_y)
                    
                    # 第二步：强制回到绝对起笔位置
                    self.robot.move_to_start_position()
                    
                    # 第三步：开始写字
                    # self.robot.write_shi()
                    self.robot.write_dept_logo() # 这里我把写字恢复了，你可以按需注释
                    
                    rospy.loginfo("🎉 全部流程执行完毕！")
                    cv2.destroyAllWindows()
                    rospy.signal_shutdown("任务完成退出")
                    return

if __name__ == '__main__':
    try:
        rospy.init_node('robot_main_node', anonymous=True)
        manager = RobotManager()
        manager.run_process()
    except rospy.ROSInterruptException:
        pass