#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import numpy as np

from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

import tf2_ros
import tf.transformations as tft

from test2 import KinovaCalligraphy
import yaml


class RobotManager:
    def __init__(self):
        self.robot = KinovaCalligraphy()
        self.HOVER_ONLY = rospy.get_param("~hover_only", True)

        # =========================
        # 基础坐标系参数
        # =========================
        self.BASE_FRAME = rospy.get_param("~base_frame", "base_link")
        self.CAMERA_FRAME = "usb_cam"
        self.IMAGE_TOPIC = "/usb_cam/image_raw"
        self.CAMERA_INFO_TOPIC = "/usb_cam/camera_info"

        # =========================
        # YOLO 参数
        # =========================
        default_model_path = "/home/lc/catkin_ws/src/calligraphy_project/scripts/best.pt"
        self.MODEL_PATH = rospy.get_param("~model_path", default_model_path)

        self.CONF_THRES = rospy.get_param("~conf_thres", 0.35)
        self.IMGSZ = rospy.get_param("~imgsz", 640)

        # 连续取多帧，取中位数，减少单帧抖动
        self.N_GOOD_FRAMES = rospy.get_param("~n_good_frames", 8)
        self.MAX_TRY_FRAMES = rospy.get_param("~max_try_frames", 100)

        # =========================
        # 核心：墨水盒口平面高度
        # =========================
        # 这个值是 base_link 坐标系下的 z 坐标，单位 m。
        # 你的 test2.py 里 Z_PRESS_DEEP = 0.148，所以这里默认先用 0.148。
        self.Z_INK_PLANE = rospy.get_param("~z_ink_plane", self.robot.Z_PRESS_DEEP)

        # =========================
        # 最后毫米级微调
        # =========================
        # 如果算出来后总是偏一点点，用这个调。
        # 注意单位是米，0.005 = 5 mm。
        self.FINE_OFFSET_X = rospy.get_param("~fine_offset_x", 0.0)
        self.FINE_OFFSET_Y = rospy.get_param("~fine_offset_y", -0)

        # 如果 YOLO 框中心不是墨水盒口中心，可在像素层面补偿
        self.PIXEL_OFFSET_U = rospy.get_param("~pixel_offset_u", 0.0)
        self.PIXEL_OFFSET_V = rospy.get_param("~pixel_offset_v", 0.0)

        # 安全限制，防止误识别后一口气飞太远
        self.MAX_MOVE_XY = rospy.get_param("~max_move_xy", 0.25)

        # 是否只测试定位，不真的执行蘸墨
        self.DRY_RUN = rospy.get_param("~dry_run", False)

        rospy.loginfo("========== 参数检查 ==========")
        rospy.loginfo(f"BASE_FRAME       = {self.BASE_FRAME}")
        rospy.loginfo(f"CAMERA_FRAME     = {self.CAMERA_FRAME}")
        rospy.loginfo(f"IMAGE_TOPIC      = {self.IMAGE_TOPIC}")
        rospy.loginfo(f"CAMERA_INFO_TOPIC= {self.CAMERA_INFO_TOPIC}")
        rospy.loginfo(f"MODEL_PATH       = {self.MODEL_PATH}")
        rospy.loginfo(f"Z_INK_PLANE      = {self.Z_INK_PLANE:.4f}")
        rospy.loginfo(f"FINE_OFFSET_X    = {self.FINE_OFFSET_X:.4f}")
        rospy.loginfo(f"FINE_OFFSET_Y    = {self.FINE_OFFSET_Y:.4f}")
        rospy.loginfo("==============================")

        # =========================
        # ROS 工具
        # =========================
        self.bridge = CvBridge()
        self.latest_frame = None
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        ok = self.load_handeye_from_yaml()
        if not ok:
            raise RuntimeError("手眼标定 YAML 加载失败，无法继续。")

        rospy.Subscriber(self.IMAGE_TOPIC, Image, self.image_callback, queue_size=1)
        rospy.Subscriber(self.CAMERA_INFO_TOPIC, CameraInfo, self.camera_info_callback, queue_size=1)

        rospy.loginfo(f"正在加载 YOLO 模型: {self.MODEL_PATH}")
        self.model = YOLO(self.MODEL_PATH)

        rospy.loginfo("RobotManager 初始化完成。")
    def load_handeye_from_yaml(self):
        """从 easy_handeye 保存的 yaml 文件中读取手眼标定矩阵"""

        yaml_path = "/home/lc/.ros/easy_handeye/easy_handeye_eye_on_hand.yaml"

        if not os.path.exists(yaml_path):
            rospy.logerr("找不到手眼标定文件: {}".format(yaml_path))
            return False

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        tf_data = data["transformation"]

        qx = float(tf_data["qx"])
        qy = float(tf_data["qy"])
        qz = float(tf_data["qz"])
        qw = float(tf_data["qw"])

        tx = float(tf_data["x"])
        ty = float(tf_data["y"])
        tz = float(tf_data["z"])

        self.T_ee_cam = tft.quaternion_matrix([qx, qy, qz, qw])
        self.T_ee_cam[0, 3] = tx
        self.T_ee_cam[1, 3] = ty
        self.T_ee_cam[2, 3] = tz

        rospy.loginfo("成功从 YAML 加载手眼标定矩阵 T_ee_cam:")
        rospy.loginfo("T_ee_cam =\n{}".format(self.T_ee_cam))

        rospy.loginfo("成功加载 T_ee_cam")

        return True
    def pose_to_matrix(self, pose):
        """
        geometry_msgs/Pose -> 4x4 齐次变换矩阵
        """
        q = pose.orientation
        t = pose.position

        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        return T
    def get_T_base_cam(self):
        """
        计算当前相机坐标系到 base_link 坐标系的变换：
        T_base_cam = T_base_ee · T_ee_cam
        """

        if not hasattr(self, "T_ee_cam"):
            rospy.logerr("未加载 T_ee_cam，请先调用 load_handeye_from_yaml()。")
            return None

        ee_pose = self.robot.move_group.get_current_pose("end_effector_link").pose
        T_base_ee = self.pose_to_matrix(ee_pose)

        T_base_cam = T_base_ee @ self.T_ee_cam

        rospy.loginfo_throttle(1.0, "T_base_ee =\n{}".format(T_base_ee))
        rospy.loginfo_throttle(1.0, "T_base_cam =\n{}".format(T_base_cam))

        return T_base_cam

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def wait_for_ready(self):
        rospy.loginfo("等待图像、相机内参和 YAML 手眼标定矩阵...")

        # =========================
        # 1. 等待摄像头图像
        # =========================
        while not rospy.is_shutdown() and self.latest_frame is None:
            rospy.loginfo_throttle(1.0, f"等待图像: {self.IMAGE_TOPIC}")
            rospy.sleep(0.1)

        # =========================
        # 2. 等待相机内参 CameraInfo
        # =========================
        while not rospy.is_shutdown() and self.camera_info is None:
            rospy.loginfo_throttle(1.0, f"等待 camera_info: {self.CAMERA_INFO_TOPIC}")
            rospy.sleep(0.1)

        # =========================
        # 3. 检查 YAML 手眼标定矩阵 T_ee_cam 是否已经加载
        # =========================
        if not hasattr(self, "T_ee_cam"):
            rospy.logerr("未检测到 YAML 手眼标定矩阵 T_ee_cam。")
            rospy.logerr("请确认 __init__() 中已经调用 self.load_handeye_from_yaml()。")
            raise RuntimeError("YAML 手眼标定矩阵 T_ee_cam 未加载。")

        rospy.loginfo("YAML 手眼标定矩阵 T_ee_cam 已加载。")
        rospy.loginfo("T_ee_cam =\n{}".format(self.T_ee_cam))

        rospy.loginfo("系统准备完成。")

    def detect_inkbox_once(self, frame_bgr):
        results = self.model(
            frame_bgr,
            imgsz=self.IMGSZ,
            conf=self.CONF_THRES,
            verbose=False
        )

        result = results[0]
        boxes = result.boxes

        debug_frame = result.plot()

        if boxes is None or len(boxes) == 0:
            return None, debug_frame

        # 选置信度最高的框
        best_idx = int(boxes.conf.argmax().item())
        conf = float(boxes.conf[best_idx].item())

        x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy()

        u = float((x1 + x2) / 2.0 + self.PIXEL_OFFSET_U)
        v = float((y1 + y2) / 2.0 + self.PIXEL_OFFSET_V)

        # 画中心点和图像中心线
        h, w = debug_frame.shape[:2]
        cv2.line(debug_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        cv2.line(debug_frame, (0, h // 2), (w, h // 2), (255, 0, 0), 1)
        cv2.circle(debug_frame, (w // 2, h // 2), 4, (255, 0, 0), -1)
        cv2.circle(debug_frame, (int(u), int(v)), 6, (0, 0, 255), -1)

        cv2.putText(
            debug_frame,
            f"u={u:.1f}, v={v:.1f}, conf={conf:.2f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        det = {
            "u": u,
            "v": v,
            "conf": conf,
            "xyxy": (x1, y1, x2, y2)
        }

        return det, debug_frame

    def pixel_to_base_on_plane(self, u, v):
        """
        像素点 u,v -> base_link 下的 3D 点。

        方法：
        1. 利用相机内参 K 和畸变参数 D，将像素点去畸变；
        2. 将像素点反投影为 usb_cam 坐标系下的一条空间射线 ray_cam；
        3. 使用 YAML 中读取的手眼标定矩阵，将 ray_cam 转换到 base_link 坐标系；
        4. 令该射线与 z = Z_INK_PLANE 的墨水盒平面求交；
        5. 得到墨水盒目标点在 base_link 下的 3D 坐标。
        """

        if self.camera_info is None:
            rospy.logerr("camera_info 为空，无法进行像素反投影。")
            return None

        # =========================
        # 1. 读取相机内参和畸变参数
        # =========================
        K = np.array(self.camera_info.K, dtype=np.float64).reshape(3, 3)
        D = np.array(self.camera_info.D, dtype=np.float64)

        pixel = np.array([[[u, v]]], dtype=np.float64)

        # =========================
        # 2. 去畸变，得到归一化相机坐标
        # =========================
        undistorted = cv2.undistortPoints(pixel, K, D)

        x_norm = undistorted[0, 0, 0]
        y_norm = undistorted[0, 0, 1]

        # =========================
        # 3. 构造 usb_cam 坐标系下的射线
        # =========================
        ray_cam = np.array([x_norm, y_norm, 1.0], dtype=np.float64)
        ray_cam = ray_cam / np.linalg.norm(ray_cam)

        # =========================
        # 4. 使用 YAML 中加载的手眼标定结果
        # =========================
        if not hasattr(self, "T_ee_cam"):
            rospy.logerr("未加载 T_ee_cam，请先调用 load_handeye_from_yaml()。")
            return None
        T_base_cam = self.get_T_base_cam()
        if T_base_cam is None:
            return None

        origin_base = T_base_cam[:3, 3]
        R_base_cam = T_base_cam[:3, :3]

        ray_base = R_base_cam @ ray_cam

        # =========================
        # 5. 与墨水盒平面 z = Z_INK_PLANE 求交
        # =========================
        if abs(ray_base[2]) < 1e-8:
            rospy.logerr("相机射线几乎平行于墨水盒平面，无法求交。")
            rospy.logerr(f"ray_base={ray_base}")
            return None

        scale = (self.Z_INK_PLANE - origin_base[2]) / ray_base[2]

        if scale < 0:
            rospy.logerr("交点在相机后方，通常是手眼矩阵方向、camera_frame 或 z_ink_plane 有问题。")
            rospy.logerr(f"origin_base={origin_base}")
            rospy.logerr(f"ray_base={ray_base}")
            rospy.logerr(f"Z_INK_PLANE={self.Z_INK_PLANE:.4f}")
            rospy.logerr(f"scale={scale}")
            return None

        p_base = origin_base + scale * ray_base

        rospy.loginfo_throttle(
            1.0,
            "pixel=({:.1f}, {:.1f}) -> base=({:.4f}, {:.4f}, {:.4f})".format(
                u, v, p_base[0], p_base[1], p_base[2]
            )
        )

        return p_base
    def collect_stable_target(self):
        points = []
        pixels = []

        rate = rospy.Rate(10)
        try_count = 0

        rospy.loginfo("开始连续识别墨水盒...")

        while not rospy.is_shutdown() and try_count < self.MAX_TRY_FRAMES:
            try_count += 1

            if self.latest_frame is None:
                rate.sleep()
                continue

            frame = self.latest_frame.copy()
            det, debug_frame = self.detect_inkbox_once(frame)

            if det is None:
                rospy.loginfo_throttle(1.0, "未检测到墨水盒，继续寻找...")
                cv2.imshow("Robot Vision: Ink Detection", debug_frame)
                cv2.waitKey(1)
                rate.sleep()
                continue

            u = det["u"]
            v = det["v"]

            p_base = self.pixel_to_base_on_plane(u, v)

            if p_base is None:
                rospy.logwarn("像素点转 base_link 坐标失败，跳过本帧。")
                cv2.imshow("Robot Vision: Ink Detection", debug_frame)
                cv2.waitKey(1)
                rate.sleep()
                continue

            points.append(p_base)
            pixels.append([u, v])

            rospy.loginfo(
                f"有效检测 {len(points)}/{self.N_GOOD_FRAMES}: "
                f"pixel=({u:.1f}, {v:.1f}), "
                f"base=({p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f}), "
                f"conf={det['conf']:.2f}"
            )

            cv2.putText(
                debug_frame,
                f"base x={p_base[0]:.3f}, y={p_base[1]:.3f}, z={p_base[2]:.3f}",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2
            )

            cv2.imshow("Robot Vision: Ink Detection", debug_frame)
            cv2.waitKey(1)

            if len(points) >= self.N_GOOD_FRAMES:
                break

            rate.sleep()

        if len(points) < max(3, self.N_GOOD_FRAMES // 2):
            rospy.logerr(f"有效检测帧太少：{len(points)}，放弃蘸墨。")
            return None

        points = np.array(points, dtype=np.float64)
        pixels = np.array(pixels, dtype=np.float64)

        target = np.median(points, axis=0)
        target_pixel = np.median(pixels, axis=0)

        target[0] += self.FINE_OFFSET_X
        target[1] += self.FINE_OFFSET_Y
        target[2] = self.Z_INK_PLANE

        rospy.loginfo("========== 墨水盒最终定位 ==========")
        rospy.loginfo(f"像素中心中位数: u={target_pixel[0]:.2f}, v={target_pixel[1]:.2f}")
        rospy.loginfo(f"base_link 目标点: x={target[0]:.4f}, y={target[1]:.4f}, z={target[2]:.4f}")
        rospy.loginfo(f"世界坐标微调: dx={self.FINE_OFFSET_X:.4f}, dy={self.FINE_OFFSET_Y:.4f}")
        rospy.loginfo("===================================")

        return target

    def get_current_ee_pose(self):
        pose_stamped = self.robot.move_group.get_current_pose("end_effector_link")
        rospy.loginfo("MoveIt current_pose frame_id = {}".format(pose_stamped.header.frame_id))
        return pose_stamped.pose

    def execute_dip_to_target(self, target_base):
        target_x = float(target_base[0])
        target_y = float(target_base[1])
        target_z = float(target_base[2])

        rospy.loginfo("========== 蘸墨目标点 ==========")
        rospy.loginfo(f"墨水盒目标点 base_link: x={target_x:.4f}, y={target_y:.4f}, z={target_z:.4f}")
        rospy.loginfo("现在使用绝对坐标控制，不再使用相对 move_x / move_y")
        rospy.loginfo("================================")

        if self.DRY_RUN:
            rospy.logwarn("DRY_RUN=True，只打印目标点，不执行机械臂运动。")
            return True

        if self.HOVER_ONLY:
            rospy.logwarn("HOVER_ONLY=True，只移动到墨水盒上方，不下压蘸墨。")
            self.robot.hover_to_base_xy(target_x, target_y)
            return True

        self.robot.dip_ink_to_base_xy(target_x, target_y)
        return True

    # def execute_dip_to_target(self, target_base):
    #     current_pose = self.get_current_ee_pose()

    #     current_x = current_pose.position.x
    #     current_y = current_pose.position.y
    #     current_z = current_pose.position.z

    #     target_x = float(target_base[0])
    #     target_y = float(target_base[1])

    #     move_x = target_x - current_x
    #     move_y = target_y - current_y

    #     rospy.loginfo("========== 蘸墨运动计算 ==========")
    #     rospy.loginfo(f"当前末端位置: x={current_x:.4f}, y={current_y:.4f}, z={current_z:.4f}")
    #     rospy.loginfo(f"墨水盒目标点: x={target_x:.4f}, y={target_y:.4f}, z={target_base[2]:.4f}")
    #     rospy.loginfo(f"传给 dip_ink_action 的相对位移: move_x={move_x:.4f}, move_y={move_y:.4f}")
    #     rospy.loginfo("=================================")

    #     if abs(move_x) > self.MAX_MOVE_XY or abs(move_y) > self.MAX_MOVE_XY:
    #         rospy.logerr("相对移动量过大，疑似识别/标定错误，为安全起见不执行。")
    #         rospy.logerr(f"move_x={move_x:.4f}, move_y={move_y:.4f}, MAX_MOVE_XY={self.MAX_MOVE_XY:.4f}")
    #         return False

    #     if self.DRY_RUN:
    #         rospy.logwarn("DRY_RUN=True，只打印结果，不执行机械臂蘸墨。")
    #         return True

    #     self.robot.dip_ink_action(move_x, move_y)
    #     return True

    def run_process(self):
        self.wait_for_ready()

        planning_frame = self.robot.move_group.get_planning_frame()
        rospy.loginfo(f"MoveIt planning frame = {planning_frame}")
        if planning_frame != self.BASE_FRAME:
            rospy.logwarn(
                f"注意：MoveIt planning frame 是 {planning_frame}，"
                f"但视觉定位使用的是 {self.BASE_FRAME}。如果后面偏差很大，需要统一这两个坐标系。"
            )

        rospy.loginfo("机械臂移动到鹰眼扫描位姿...")
        self.robot.move_to_scan_position()
        rospy.sleep(2.0)

        target_base = self.collect_stable_target()
        if target_base is None:
            rospy.logerr("墨水盒定位失败，任务终止。")
            cv2.destroyAllWindows()
            return

        ok = self.execute_dip_to_target(target_base)
        if not ok:
            rospy.logerr("蘸墨动作未执行。")
            cv2.destroyAllWindows()
            return

        rospy.loginfo("蘸墨完成，回到起笔位置...")
        self.robot.move_to_start_position()

        # 需要写字时再取消注释
        # self.robot.write_shi()
        # self.robot.write_dept_logo()

        rospy.loginfo("全部流程执行完成。")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        rospy.init_node("robot_main_node", anonymous=True)
        manager = RobotManager()
        manager.run_process()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常退出: {e}")
        cv2.destroyAllWindows()