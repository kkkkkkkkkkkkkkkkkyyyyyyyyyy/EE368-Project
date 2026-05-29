#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import math
import tf.transformations

class KinovaCalligraphy:
    def __init__(self):
        # 1. 初始化 MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kinova_strokes_node', anonymous=True)

        self.group_name = "arm"
        
        # 【关键修改】明确告诉 MoveIt 去找带命名空间的参数！
       # 【终极修改】同时指定参数路径(robot_description)和动作服务端命名空间(ns)
        robot_description = "my_gen3_lite/robot_description"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name, robot_description=robot_description, ns="my_gen3_lite")

        # 降速运行，安全第一
        self.move_group.set_max_velocity_scaling_factor(0.08)
        self.move_group.set_max_acceleration_scaling_factor(0.08)

        # ====================================================
        # 2. 关键高度参数配置 (请根据你图片中测量的数据进行修改)
        # ====================================================
        self.Z_HOVER = 0.175       # 安全悬停高度（笔尖离开纸面）
        self.Z_PRESS_DEEP = 0.148  # 深压高度（用于写粗笔画、起笔顿笔）
        self.Z_PRESS_NORM = 0.156  # 标准行笔高度（普通笔画粗细）
        self.Z_PRESS_LIGHT = 0.157 # 轻触高度（用于收笔出锋，笔尖刚好擦到纸）
        # ====================================================

        rospy.loginfo("Kinova 基础笔画库初始化成功！")
    def dip_ink_action(self, ink_tray_x, ink_tray_y):
            """
            蘸水动作：从当前位置移动到墨盘位置 -> 下潜 -> 抬起
            ink_tray_x, ink_tray_y 是从 YOLO 传来的绝对坐标（或者根据标定转换后的坐标）
            """
            rospy.loginfo("正在移动到墨盘位置进行蘸水...")
            waypoints = []
            wpose = copy.deepcopy(self.anchor_pose)
            
            # 1. 移动到墨盘上方 (这里的 Z_HOVER 是你的安全高度)
            wpose.position.x = ink_tray_x
            wpose.position.y = ink_tray_y
            wpose.position.z = self.Z_HOVER 
            waypoints.append(copy.deepcopy(wpose))
            
            # 2. 下潜蘸水
            wpose.position.z = self.Z_PRESS_DEEP # 假设这个深度能接触到墨水
            waypoints.append(copy.deepcopy(wpose))
            
            # 3. 停留蘸水
            rospy.sleep(0.8)
            
            # 4. 抬起并回到安全高度
            wpose.position.z = self.Z_HOVER
            waypoints.append(copy.deepcopy(wpose))
            
            self.execute_cartesian_path(waypoints)
            rospy.loginfo("蘸水完成！")
    def execute_cartesian_path(self, waypoints):
        """执行笛卡尔空间直线轨迹"""
        # 每 2 毫米插入一个控制点，保证轨迹丝滑
        # 【修改这里】：显式指定参数名称，避免 Noetic 下的 C++ 签名重载冲突
        # 直接按顺序传入：路点、步长(0.002m)、开启避障(True)
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.002, True)
        if fraction > 0.95:
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return True
        else:
            rospy.logerr("轨迹规划失败，可能超出工作空间或遭遇奇异点。")
            return False

    def get_base_pose(self, start_x, start_y):
        """获取标准姿态基准，强行锁定为 Web App 中记录的完美下笔姿态"""
        pose = geometry_msgs.msg.Pose()
        
        # 1. 设定位置 (X, Y 为传入的笔画起点，Z 始终保持安全悬停高度)
        pose.position.x = start_x
        pose.position.y = start_y
        pose.position.z = self.Z_HOVER

        # 2. 设定完美姿态 (将图片里的角度从 度 转换为 弧度)
        roll = math.radians(12.1)
        pitch = math.radians(162.2)
        yaw = math.radians(80.5)

        # 3. 使用 tf 库将欧拉角转换为四元数
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def draw_heng(self, start_x, start_y, length=0.08):
        """【横 - 一】逻辑：从左往右划，Z轴保持标准深度"""
        rospy.loginfo("正在书写：横 (一)")
        wpose = self.get_base_pose(start_x, start_y)
        waypoints = []

        # 1. 悬停在起点上方 -> 2. 落笔顿笔 -> 3. 行笔 -> 4. 提笔
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.z = self.Z_PRESS_DEEP # 起笔稍重
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.y += length          # 向右行笔 (ROS机械臂坐标系中+Y通常为左，请根据实际基座朝向调整正负)
        wpose.position.z = self.Z_PRESS_NORM
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.z = self.Z_HOVER      # 垂直起笔
        waypoints.append(copy.deepcopy(wpose))

        self.execute_cartesian_path(waypoints)

    def draw_shu(self, start_x, start_y, length=0.08):
        """【竖 - 丨】逻辑：从上往下（朝向基座方向）划"""
        rospy.loginfo("正在书写：竖 (丨)")
        wpose = self.get_base_pose(start_x, start_y)
        waypoints = []

        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x -= length          # 朝基座拉近
        wpose.position.z = self.Z_PRESS_NORM
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))

        self.execute_cartesian_path(waypoints)

    def draw_pie(self, start_x, start_y, length=0.06):
        """【撇 - 丿】核心创新：斜向移动的同时，Z轴逐渐抬高，让笔画由粗变细出锋"""
        rospy.loginfo("正在书写：撇 (丿) [包含动态形变控制]")
        wpose = self.get_base_pose(start_x, start_y)
        waypoints = []

        # 1. 悬停起点
        waypoints.append(copy.deepcopy(wpose))
        # 2. 顿笔起笔（深压）
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        
        # 3. 动态行笔（分成多段，边走边抬高 Z 轴）
        steps = 10
        dx = -length / steps       # 向下移动
        dy = length * 0.8 / steps  # 向左斜向移动
        dz = (self.Z_PRESS_LIGHT - self.Z_PRESS_DEEP) / steps # 逐渐变浅

        for _ in range(steps):
            wpose.position.x += dx
            wpose.position.y += dy
            wpose.position.z += dz
            waypoints.append(copy.deepcopy(wpose))
        
        # 4. 起笔撤离
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))

        self.execute_cartesian_path(waypoints)

    def draw_dian(self, start_x, start_y):
        """【点 - 丶】逻辑：斜向下压后迅速提笔"""
        rospy.loginfo("正在书写：点 (丶)")
        wpose = self.get_base_pose(start_x, start_y)
        waypoints = []

        waypoints.append(copy.deepcopy(wpose))
        
        # 向右下方切入并压深
        wpose.position.x -= 0.01
        wpose.position.y += 0.01
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        
        # 提起
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))

        self.execute_cartesian_path(waypoints)

if __name__ == '__main__':
    try:
        robot = KinovaCalligraphy()
        
        # 极其重要：请在工作空间内挑选一个安全的空旷起始点（单位：米）
        # 先让机器人在纸面上方空写测试！
        # 使用 Web App 中真实的当前位置作为测试中心点 (单位：米)
        test_x = 0.178
        test_y = 0.135
        
        rospy.sleep(2) # 等待系统稳定
        
        # 依次测试四个基本笔画，每个笔画之间错开位置防止重叠
        robot.draw_heng(test_x, test_y - 0.05)
        rospy.sleep(1)
        
        robot.draw_shu(test_x, test_y + 0.05)
        rospy.sleep(1)
        
        robot.draw_pie(test_x + 0.05, test_y)
        rospy.sleep(1)
        
        robot.draw_dian(test_x + 0.05, test_y - 0.05)
        
        rospy.loginfo("所有单笔画测试完成！")

    except rospy.ROSInterruptException:
        pass
