#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg

class KinovaCalligraphy:
    def __init__(self):
        # 1. 初始化 MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kinova_strokes_node', anonymous=True)

        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # 降速运行，安全第一
        self.move_group.set_max_velocity_scaling_factor(0.08)
        self.move_group.set_max_acceleration_scaling_factor(0.08)

        # ====================================================
        # 2. 关键高度参数配置 (请根据你图片中测量的数据进行修改)
        # ====================================================
        self.Z_HOVER = 0.175       # 安全悬停高度（笔尖离开纸面）
        self.Z_PRESS_DEEP = 0.148  # 深压高度（用于写粗笔画、起笔顿笔）
        self.Z_PRESS_NORM = 0.156  # 标准行笔高度（普通笔画粗细）
        self.Z_PRESS_LIGHT = 0.150 # 轻触高度（用于收笔出锋，笔尖刚好擦到纸）
        # ====================================================

        rospy.loginfo("Kinova 基础笔画库初始化成功！")

    def execute_cartesian_path(self, waypoints):
        """执行笛卡尔空间直线轨迹"""
        # 每 2 毫米插入一个控制点，保证轨迹丝滑
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.002, 0.0)
        if fraction > 0.95:
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return True
        else:
            rospy.logerr("轨迹规划失败，可能超出工作空间或遭遇奇异点。")
            return False

    def get_base_pose(self, start_x, start_y):
        """获取标准姿态基准，锁定夹爪完全垂直向下"""
        pose = self.move_group.get_current_pose().pose
        pose.position.x = start_x
        pose.position.y = start_y
        pose.position.z = self.Z_HOVER
        # 注意：此处保持了机器人当前的手腕翻转姿态（建议手动将笔调至垂直桌面）
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
        test_x = 0.35
        test_y = 0.00
        
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
