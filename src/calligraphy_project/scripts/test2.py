#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import math
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf.transformations as tf

class KinovaCalligraphy:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        # 注意：这里去掉了 rospy.init_node，由外部 main.py 统一管理

        self.group_name = "arm"
        robot_description = "my_gen3_lite/robot_description"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name, robot_description=robot_description, ns="my_gen3_lite")
        self.move_group.set_pose_reference_frame("base_link")
        self.move_group.set_end_effector_link("end_effector_link")

        rospy.loginfo("MoveIt planning_frame: {}".format(self.move_group.get_planning_frame()))
        rospy.loginfo("MoveIt pose_reference_frame: {}".format(self.move_group.get_pose_reference_frame()))
        rospy.loginfo("MoveIt end_effector_link: {}".format(self.move_group.get_end_effector_link()))
        # 速度调至 20%，保证平稳，降低 CONTROL_FAILED 概率
        self.move_group.set_max_velocity_scaling_factor(0.2)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        
        self.Z_HOVER = 0.135     # 安全悬停高度
        self.Z_PRESS_DEEP = 0.140 # 深压高度
        self.Z_PRESS_NORM = 0.141 # 标准行笔高度
        self.Z_PRESS_LIGHT = 0.143 # 轻触高度

        # ==========================================
        # 【核心策略：绝对静态锚点锁定】
        # 直接写入你截图中的完美数据，防止由于机械臂细微抖动导致姿态算错
        # ==========================================
        rospy.loginfo("正在锁定静态绝对锚点 (基于 UI 截图数据)...")
        self.anchor_pose = Pose()
        
        # 1. 填入截图中的 Linear (cm 转 m)
        self.anchor_pose.position.x = 0.387  # 38.7 cm
        self.anchor_pose.position.y = -0.080 # -8.0 cm
        self.anchor_pose.position.z = 0.173  # 17.3 cm
        
        # 2. 填入截图中的 Angular (度转弧度，再转四元数)
        roll = math.radians(18.3)
        pitch = math.radians(173.0)
        yaw = math.radians(82.0)
        
        q = tf.quaternion_from_euler(roll, pitch, yaw)
        self.anchor_pose.orientation.x = q[0]
        self.anchor_pose.orientation.y = q[1]
        self.anchor_pose.orientation.z = q[2]
        self.anchor_pose.orientation.w = q[3]
        
        rospy.loginfo("锚点抓取成功！位置与手腕姿态已永久锁死！")
    def hover_to_base_xy(self, target_x, target_y):
        """只悬停到墨水盒上方，不下压，用来验证准不准"""
        rospy.loginfo("正在悬停到墨水盒目标点上方...")
        waypoints = []

        wpose = self.move_group.get_current_pose("end_effector_link").pose
        wpose = copy.deepcopy(wpose)

        wpose.position.x = target_x
        wpose.position.y = target_y
        wpose.position.z = self.Z_HOVER

        waypoints.append(copy.deepcopy(wpose))
        self.execute_cartesian_path(waypoints)

        rospy.loginfo("已悬停到目标点上方。")


    def dip_ink_to_base_xy(self, target_x, target_y):
        """使用 base_link 下的绝对 x/y 执行蘸墨，不再用相对偏移"""
        rospy.loginfo("正在执行绝对坐标蘸墨动作...")
        waypoints = []

        wpose = self.move_group.get_current_pose("end_effector_link").pose
        wpose = copy.deepcopy(wpose)

        # 1. 直接移动到墨水盒目标点上方
        wpose.position.x = target_x
        wpose.position.y = target_y
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))

        # 2. 下压蘸墨
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))

        # 3. 抬起
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))

        self.execute_cartesian_path(waypoints)

        rospy.sleep(1.0)
        rospy.loginfo("绝对坐标蘸墨完成！")

    def execute_cartesian_path(self, waypoints):
        """执行笛卡尔直线，强制锁死手腕"""
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, True)
        
        # 稍微放宽一点点标准到 85%，增加容错率
        if fraction >= 0.85: 
            current_state = self.move_group.get_current_state()
            safe_plan = self.move_group.retime_trajectory(current_state, plan, 0.4, 0.4)
            
            self.move_group.execute(safe_plan, wait=True)
            
            # 【防报错核心】：每次执行完必须彻底叫停清空，防止惯性带入下一步
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(0.5) 
        else:
            rospy.logerr("轨迹规划失败 (完成度 {:.2f}%)，请检查是否超出工作范围。".format(fraction * 100))

    def move_to_start_position(self):
        """让机械臂从蘸水点回到固定的起笔位置"""
        rospy.loginfo("正在平滑移动到固定起笔点...")
        waypoints = []
        
        # 因为我们已经在 __init__ 里把起笔点设为了绝对锚点
        # 所以直接 deepcopy 就可以完美复原截图里的位置和姿态！
        wpose = copy.deepcopy(self.anchor_pose)
        
        waypoints.append(wpose)
        self.execute_cartesian_path(waypoints)
        rospy.loginfo("已到达固定起笔位，姿态同步完成，准备书写！")
    def move_to_scan_position(self):
        """移动到高处俯视桌面，寻找墨水盘"""
        rospy.loginfo("升起机械臂，进入鹰眼巡视模式...")
        waypoints = []
        wpose = copy.deepcopy(self.anchor_pose)
        
        # 这里的 Z 轴高度要设得比较高，比如 0.3 米，确保视野够大
        wpose.position.x = 0.35 
        wpose.position.y = 0.0
        wpose.position.z = 0.30 
        
        waypoints.append(wpose)
        self.execute_cartesian_path(waypoints)


    def dip_ink_action(self, move_x, move_y):
        rospy.loginfo("正在执行蘸水动作...")
        waypoints = []
        
        # ====================================================
        # 🚨 【超级核心修复】：抓取机械臂【当前所在的鹰眼位置】作为基准系！
        # 绝对不能再用 self.anchor_pose（写字起笔点）了！
        # ====================================================
        current_pose = self.move_group.get_current_pose("end_effector_link").pose
        wpose = copy.deepcopy(current_pose)
        
        # 1. 在当前悬空位置的基础上，平移视觉算出来的 (dx + OFFSET)
        wpose.position.x += move_x
        wpose.position.y += move_y
        wpose.position.z = self.Z_HOVER 
        waypoints.append(copy.deepcopy(wpose))
        
        # 2. 下潜蘸水
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        
        # 3. 抬起
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        
        self.execute_cartesian_path(waypoints)
        
        rospy.sleep(1.0)
        rospy.loginfo("蘸水完成！")

    # ================= 以下是书写函数，保持原样逻辑 =================
    def draw_heng(self, offset_x, offset_y, length=0.08):
        rospy.loginfo("正在书写：横 (一)")
        waypoints = []
        wpose = copy.deepcopy(self.anchor_pose)
        wpose.position.x += offset_x
        wpose.position.y += offset_y
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_PRESS_DEEP 
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += length          
        wpose.position.z = self.Z_PRESS_NORM
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_HOVER      
        waypoints.append(copy.deepcopy(wpose))
        self.execute_cartesian_path(waypoints)

    def draw_shu(self, offset_x, offset_y, length=0.08):
        rospy.loginfo("正在书写：竖 (丨)")
        waypoints = []
        wpose = copy.deepcopy(self.anchor_pose)
        wpose.position.x += offset_x
        wpose.position.y += offset_y
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= length          
        wpose.position.z = self.Z_PRESS_NORM
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        self.execute_cartesian_path(waypoints)

    def draw_pie(self, offset_x, offset_y, length=0.06):
        rospy.loginfo("正在书写：撇 (丿)")
        waypoints = []
        wpose = copy.deepcopy(self.anchor_pose)
        wpose.position.x += offset_x
        wpose.position.y += offset_y
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        steps = 10
        dx = -length / steps       
        dy = length * 0.8 / steps  
        dz = (self.Z_PRESS_LIGHT - self.Z_PRESS_DEEP) / steps 
        for _ in range(steps):
            wpose.position.x += dx
            wpose.position.y += dy
            wpose.position.z += dz
            waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        self.execute_cartesian_path(waypoints)
        
    def write_shi(self):
        rospy.loginfo("开始书写汉字：【十】")
        stroke_length = 0.08
        half_length = stroke_length / 2.0
        self.draw_heng(offset_x=0.0, offset_y=-0.02, length=stroke_length)
        rospy.sleep(0.5) 
        self.draw_shu(offset_x=half_length, offset_y=0.02, length=stroke_length)
        rospy.loginfo("【十】字完美对齐书写完成！")

    def draw_dian(self, offset_x, offset_y):
        rospy.loginfo("正在书写：点 (丶)")
        waypoints = []
        wpose = copy.deepcopy(self.anchor_pose)
        wpose.position.x += offset_x
        wpose.position.y += offset_y
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= 0.01
        wpose.position.y += 0.01
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        self.execute_cartesian_path(waypoints)

    def write_dept_logo(self):
        rospy.loginfo("🔥 开始书写电子系徽图案...")
        delay = 0.3
        self.draw_heng(offset_x=0.05, offset_y=-0.03, length=0.06)
        rospy.sleep(delay)
        self.draw_shu(offset_x=0.03, offset_y=0.03, length=0.025)
        self.draw_heng(offset_x=0.03, offset_y=0.01, length=0.02)
        rospy.sleep(delay)
        self.draw_shu(offset_x=0.03, offset_y=-0.03, length=0.025)
        self.draw_heng(offset_x=0.03, offset_y=-0.03, length=0.02)
        rospy.sleep(delay)
        self.draw_dian(offset_x=0.03, offset_y=0.012) 
        self.draw_dian(offset_x=0.03, offset_y=-0.021)
        self.draw_dian(offset_x=0.02, offset_y=0.012) 
        self.draw_dian(offset_x=0.02, offset_y=-0.021) 
        rospy.sleep(delay)
        self.draw_shu(offset_x=-0.01, offset_y=0.03, length=0.04)   
        self.draw_heng(offset_x=-0.01, offset_y=0.01, length=0.02)  
        self.draw_heng(offset_x=-0.03, offset_y=0.01, length=0.02)  
        self.draw_heng(offset_x=-0.05, offset_y=0.01, length=0.02)  
        rospy.sleep(delay)
        self.draw_shu(offset_x=-0.01, offset_y=-0.03, length=0.04)  
        self.draw_heng(offset_x=-0.01, offset_y=-0.03, length=0.02) 
        self.draw_heng(offset_x=-0.03, offset_y=-0.03, length=0.02) 
        self.draw_heng(offset_x=-0.05, offset_y=-0.03, length=0.02) 
        rospy.loginfo("    压轴绝杀：正在书写带横钩的长竖主干...")
        wpose = copy.deepcopy(self.anchor_pose)
        waypoints = []
        wpose.position.x += 0.04
        wpose.position.y += 0.0
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_PRESS_DEEP
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= 0.115
        wpose.position.z = self.Z_PRESS_NORM - 0.003 
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.04 
        wpose.position.z = self.Z_PRESS_LIGHT 
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += 0.01 
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        self.execute_cartesian_path(waypoints)
        rospy.loginfo("🎉 系徽图案绘制完成！")

# =========================================================================
# 用于单独测试本文件 (如果你用 robot_main.py 运行，这部分不会冲突)
# =========================================================================
if __name__ == '__main__':
    try:
        rospy.init_node('kinova_test_node', anonymous=True)
        robot = KinovaCalligraphy()
        rospy.sleep(1) 
        
        # 调试小建议：你可以先取消注释下面这行，测试它能不能乖乖回到原点
        # robot.move_to_start_position()
        
        robot.write_dept_logo()
        rospy.loginfo("所有测试完成！")
    except rospy.ROSInterruptException:
        pass