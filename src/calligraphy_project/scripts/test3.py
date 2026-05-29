#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import cv2
import rospy
import numpy as np

from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from test2 import KinovaCalligraphy


class FakeDetectFixedDipDemo:
    def __init__(self):
        self.robot = KinovaCalligraphy()

        # 强制 MoveIt 使用 base_link 和 end_effector_link
        self.robot.move_group.set_pose_reference_frame("base_link")
        self.robot.move_group.set_end_effector_link("end_effector_link")

        rospy.loginfo("MoveIt planning_frame: {}".format(self.robot.move_group.get_planning_frame()))
        rospy.loginfo("MoveIt pose_reference_frame: {}".format(self.robot.move_group.get_pose_reference_frame()))
        rospy.loginfo("MoveIt end_effector_link: {}".format(self.robot.move_group.get_end_effector_link()))

        # ========== YOLO 参数 ==========
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/lc/catkin_ws/src/calligraphy_project/scripts/best.pt"
        )
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.conf_thres = rospy.get_param("~conf_thres", 0.35)
        self.imgsz = rospy.get_param("~imgsz", 640)

        # 检测到连续几帧才算真的检测到
        self.need_good_frames = rospy.get_param("~need_good_frames", 3)

        # 每个点最多等多久，单位秒
        self.detect_timeout = rospy.get_param("~detect_timeout", 12.0)

        # 只悬停，不下压，用于安全测试
        self.hover_only = rospy.get_param("~hover_only", True)

        # 每个点蘸几次，一般每个点 1 次，总共 3 个点
        self.repeat_each_point = rospy.get_param("~repeat_each_point", 1)

        # ========== 三个蘸墨点 ==========
        # 这里就是你“假装视觉定位”的三个定点位置
        # 单位：米，坐标系：base_link
        #
        # 先用你日志里比较稳定的墨水盒附近坐标：
        # base_link 目标点大概 x=0.4077, y=-0.0300
        #
        # 你后面只需要微调这三个点即可。
        self.fixed_points = [
            [0.4077, -0.0300],   # 点1：中心
            [0.3077, -0.0200],   # 点2：稍微偏右/偏一侧
            [0.4077, -0.1900],   # 点3：稍微偏前/偏一侧
        ]

        # 也可以整体微调
        self.offset_x = rospy.get_param("~offset_x", 0.0)
        self.offset_y = rospy.get_param("~offset_y", 0.0)

        for p in self.fixed_points:
            p[0] += self.offset_x
            p[1] += self.offset_y

        # ========== ROS 图像订阅 ==========
        self.bridge = CvBridge()
        self.latest_frame = None

        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        rospy.loginfo("正在加载 YOLO 模型: {}".format(self.model_path))
        self.model = YOLO(self.model_path)

        rospy.loginfo("========== ==========")
        rospy.loginfo("image_topic = {}".format(self.image_topic))
        rospy.loginfo("model_path = {}".format(self.model_path))
        rospy.loginfo("conf_thres = {}".format(self.conf_thres))
        rospy.loginfo("hover_only = {}".format(self.hover_only))
       
        # for i, p in enumerate(self.fixed_points):
        #     rospy.loginfo("  P{}: x={:.4f}, y={:.4f}".format(i + 1, p[0], p[1]))
        rospy.loginfo("===================================")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("图像转换失败: {}".format(e))

    def wait_for_image(self):
        rospy.loginfo("等待相机图像: {}".format(self.image_topic))
        while not rospy.is_shutdown() and self.latest_frame is None:
            rospy.loginfo_throttle(1.0, "还没收到图像...")
            rospy.sleep(0.1)
        rospy.loginfo("已收到相机图像。")

    def detect_inkbox_trigger(self, step_index):
        """
        只检测有没有墨水盒。
        检测到后返回 True。
        注意：这里完全不使用检测框坐标，只把 YOLO 当触发器。
        """

        rospy.loginfo("========== 第 {} 个点：开始视觉检测 ==========".format(step_index + 1))
        rospy.loginfo("检测到墨水盒后，将移动到坐标 P{}。".format(step_index + 1))

        start_time = rospy.Time.now()
        good_count = 0
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.detect_timeout:
                rospy.logwarn("第 {} 个点等待检测超时，跳过这个点。".format(step_index + 1))
                return False

            if self.latest_frame is None:
                rate.sleep()
                continue

            frame = self.latest_frame.copy()

            results = self.model(
                frame,
                imgsz=self.imgsz,
                conf=self.conf_thres,
                verbose=False
            )

            result = results[0]
            boxes = result.boxes
            debug_frame = result.plot()

            detected = boxes is not None and len(boxes) > 0

            if detected:
                best_idx = int(boxes.conf.argmax().item())
                best_conf = float(boxes.conf[best_idx].item())
                good_count += 1

                x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                rospy.loginfo(
                    "第 {} 个点检测触发 {}/{}: conf={:.2f}, pixel=({:.1f}, {:.1f})".format(
                        step_index + 1,
                        good_count,
                        self.need_good_frames,
                        best_conf,
                        cx,
                        cy
                    )
                )

                cv2.putText(
                    debug_frame,
                    "DETECTED,  point P{}".format(step_index + 1),
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 255),
                    2
                )

                if good_count >= self.need_good_frames:
                    rospy.loginfo("第 {} 个点视觉检测通过，准备执行运动。".format(step_index + 1))
                    cv2.imshow("Fake Detect Fixed Dip", debug_frame)
                    cv2.waitKey(300)
                    return True

            else:
                good_count = 0
                rospy.loginfo_throttle(
                    1.0,
                    "第 {} 个点还没检测到墨水盒，继续等待...".format(step_index + 1)
                )

                cv2.putText(
                    debug_frame,
                    "Waiting for inkbox... P{}".format(step_index + 1),
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2
                )

            cv2.imshow("Fake Detect Fixed Dip", debug_frame)
            cv2.waitKey(1)

            rate.sleep()

        return False

    def move_and_dip_fixed_point(self, x, y, step_index):
        """
        真正执行点运动。
        注意：这里不使用 YOLO 检测坐标。
        """

        rospy.loginfo("========== 执行点 P{} ==========".format(step_index + 1))
        rospy.loginfo("目标坐标: x={:.4f}, y={:.4f}".format(x, y))

        # 先到点上方
        waypoints = []
        wpose = copy.deepcopy(self.robot.anchor_pose)

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = self.robot.Z_HOVER

        waypoints.append(copy.deepcopy(wpose))
        self.robot.execute_cartesian_path(waypoints)

        rospy.loginfo("已到达点 P{} 上方。".format(step_index + 1))

        if self.hover_only:
            rospy.logwarn("hover_only=True：只悬停，不下压蘸墨。")
            rospy.sleep(1.0)
            return

        for k in range(self.repeat_each_point):
            rospy.loginfo("P{}：正在执行第 {}/{} 次蘸墨。".format(
                step_index + 1,
                k + 1,
                self.repeat_each_point
            ))

            waypoints = []
            wpose = copy.deepcopy(self.robot.anchor_pose)

            # 1. 悬停
            wpose.position.x = x
            wpose.position.y = y
            wpose.position.z = self.robot.Z_HOVER
            waypoints.append(copy.deepcopy(wpose))

            # 2. 下压
            wpose.position.z = self.robot.Z_PRESS_DEEP
            waypoints.append(copy.deepcopy(wpose))

            # 3. 抬起
            wpose.position.z = self.robot.Z_HOVER
            waypoints.append(copy.deepcopy(wpose))

            self.robot.execute_cartesian_path(waypoints)
            rospy.sleep(0.6)

        rospy.loginfo("点 P{} 蘸墨完成。".format(step_index + 1))

    def run(self):
        self.wait_for_image()

        rospy.loginfo("先移动到鹰眼扫描位姿...")
        self.robot.move_to_scan_position()
        rospy.sleep(1.5)

        for i, (x, y) in enumerate(self.fixed_points):
            # 每个点之前都先开摄像头检测一下
            rospy.loginfo("========== 准备检测第 {} 个点 ==========".format(i + 1))

            ok = self.detect_inkbox_trigger(i)

            if not ok:
                rospy.logwarn("第 {} 个点没有检测通过，跳过。".format(i + 1))
                
                # 即使没检测通过，也回鹰眼位，保证流程状态统一
                rospy.loginfo("检测失败，恢复到鹰眼扫描位...")
                self.robot.move_to_scan_position()
                rospy.sleep(1.2)
                continue

            # 检测通过后，不用检测坐标，直接去点
            self.move_and_dip_fixed_point(x, y, i)

            # 每次蘸完都恢复到鹰眼位置
            rospy.loginfo("第 {} 个点动作完成，恢复到鹰眼扫描位...".format(i + 1))
            self.robot.move_to_scan_position()
            rospy.sleep(1.5)

        rospy.loginfo("三个点蘸墨流程完成，当前已在鹰眼扫描位。")

        # 如果你想最后回起笔位置，就保留下面两行；
        # 如果想最后停在鹰眼位置，就注释掉下面两行。
        # rospy.loginfo("回起笔位置...")
        # self.robot.move_to_start_position()

        cv2.destroyAllWindows()
        rospy.loginfo("演示结束。")


if __name__ == "__main__":
    try:
        rospy.init_node("fake_detect_fixed_dip_node", anonymous=True)
        demo = FakeDetectFixedDipDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("程序异常退出: {}".format(e))
        cv2.destroyAllWindows()