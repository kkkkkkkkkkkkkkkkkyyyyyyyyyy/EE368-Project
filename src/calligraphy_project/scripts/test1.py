#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander

class KinovaCalligraphy:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kinova_strokes_node', anonymous=True)

        self.group_name = "arm"
        robot_description = "my_gen3_lite/robot_description"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name, robot_description=robot_description, ns="my_gen3_lite")

        # 速度调至 10%，保证平稳
        self.move_group.set_max_velocity_scaling_factor(0.4)
        self.move_group.set_max_acceleration_scaling_factor(0.4)
        
        # # 【临时修改：空写测试】把高度全部抬高 3 厘米，防止戳桌子或超限
        # self.Z_HOVER = 0.205       # 原来 0.175
        # self.Z_PRESS_DEEP = 0.178  # 原来 0.148
        # self.Z_PRESS_NORM = 0.186  # 原来 0.156
        # self.Z_PRESS_LIGHT = 0.187 # 原来 0.157

        self.Z_HOVER = 0.175       # 安全悬停高度（笔尖离开纸面）
        self.Z_PRESS_DEEP = 0.146  # 深压高度（用于写粗笔画、起笔顿笔）
        self.Z_PRESS_NORM = 0.154  # 标准行笔高度（普通笔画粗细）
        self.Z_PRESS_LIGHT = 0.156 # 轻触高度（用于收笔出锋，笔尖刚好擦到纸）
        # 【核心策略】：直接抓取真机的完美状态作为锚点！免去一切姿态转换的坑！
        rospy.loginfo("正在抓取当前完美姿态作为绝对锚点...")
        self.anchor_pose = self.move_group.get_current_pose().pose
        rospy.loginfo("锚点抓取成功！手腕姿态已锁死！")
    def print_current_pose(self):
        pose = self.move_group.get_current_pose().pose

        rospy.loginfo("当前位姿：")
        rospy.loginfo("position:")
        rospy.loginfo("x = {:.6f}".format(pose.position.x))
        rospy.loginfo("y = {:.6f}".format(pose.position.y))
        rospy.loginfo("z = {:.6f}".format(pose.position.z))

        rospy.loginfo("orientation:")
        rospy.loginfo("qx = {:.6f}".format(pose.orientation.x))
        rospy.loginfo("qy = {:.6f}".format(pose.orientation.y))
        rospy.loginfo("qz = {:.6f}".format(pose.orientation.z))
        rospy.loginfo("qw = {:.6f}".format(pose.orientation.w))
    def execute_cartesian_path(self, waypoints):
        """执行笛卡尔直线，强制锁死手腕"""
        # 放宽步长至 5mm，减少控制器计算压力
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, True)
        
        if fraction >= 0.90:
            # 揉入平滑的时间和速度曲线，防止启动加速度过大被 Kinova 拦截
            current_state = self.move_group.get_current_state()
            safe_plan = self.move_group.retime_trajectory(current_state, plan, 0.4, 0.4)
            
            self.move_group.execute(safe_plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
        else:
            rospy.logerr("轨迹规划失败 (完成度 {:.2f}%)，请检查是否超出工作范围。".format(fraction * 100))
    def dip_ink_action(self, ink_x, ink_y):
            rospy.loginfo("正在执行蘸水动作...")
            waypoints = []
            wpose = copy.deepcopy(self.anchor_pose)
            
            # 1. 移动到墨盘上方
            wpose.position.x += ink_x
            wpose.position.y += ink_y
            wpose.position.z = self.Z_HOVER 
            waypoints.append(copy.deepcopy(wpose))
            
            # 2. 下潜蘸水
            wpose.position.z = self.Z_PRESS_DEEP
            waypoints.append(copy.deepcopy(wpose))
            rospy.sleep(0.8) # 停留吸墨
            
            # 3. 抬起
            wpose.position.z = self.Z_HOVER
            waypoints.append(copy.deepcopy(wpose))
            
            self.execute_cartesian_path(waypoints)
            rospy.loginfo("蘸水完成！")
    def move_to_start_position(self):
        """让机械臂移动到固定起笔位，并锁定固定姿态"""
        rospy.loginfo("正在移动到固定起笔点，并锁定固定姿态...")

        waypoints = []
        wpose = copy.deepcopy(self.anchor_pose)

        # =========================
        # 1. 固定起笔位置
        # =========================
        wpose.position.x = 0.413
        wpose.position.y = -0.014
        wpose.position.z = 0.174

        # =========================
        # 2. 固定起笔姿态
        # 这是你刚刚打印出来的四元数
        # =========================
        wpose.orientation.x = -0.313939
        wpose.orientation.y =  0.932775
        wpose.orientation.z =  0.165484
        wpose.orientation.w =  0.063156

        # 3. 执行移动
        waypoints.append(copy.deepcopy(wpose))
        self.execute_cartesian_path(waypoints)
        rospy.sleep(0.3)

        # =========================
        # 4. 关键：把固定起笔位姿作为后续书写原点
        # 后面的 draw_heng / draw_shu / draw_dian 都会基于它偏移
        # =========================
        self.anchor_pose = copy.deepcopy(wpose)

        rospy.loginfo("已到达固定起笔位，位置和姿态均已锁定，准备书写！")
    def draw_heng(self, offset_x, offset_y, length=0.08):
        rospy.loginfo("正在书写：横 (一)")
        waypoints = []
        wpose = copy.deepcopy(self.anchor_pose)
        
        # 1. 纯直线平移到笔画起点的正上方（空中换笔）
        wpose.position.x += offset_x
        wpose.position.y += offset_y
        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))
        
        # 2. 直线下压落笔
        wpose.position.z = self.Z_PRESS_DEEP 
        waypoints.append(copy.deepcopy(wpose))
        
        # 3. 直线行笔
        wpose.position.y += length          
        wpose.position.z = self.Z_PRESS_NORM
        waypoints.append(copy.deepcopy(wpose))
        
        # 4. 直线抬笔
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
        """书写组合汉字：十（完美居中对齐版）"""
        rospy.loginfo("开始书写汉字：【十】")
        
        stroke_length = 0.08
        half_length = stroke_length / 2.0

        # 1. 写横：起点设在偏右 2 厘米处 (offset_y = -0.02)
        # 因为它往左划 8 厘米，所以它的中心点刚好在偏左 2 厘米处 (offset_y = +0.02)
        self.draw_heng(offset_x=0.0, offset_y=-0.02, length=stroke_length)
        rospy.sleep(0.5) 
        
        # 2. 写竖：【核心修改】为了精准从横的中间穿过
        # 竖的左右位置 (offset_y) 必须死死对齐“横”的中心点，也就是 +0.02！
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
        """
        书写电子与电气工程系系徽核心图案（小篆风格的‘電’）
        整体尺寸约 10cm x 8cm，完全对称设计
        """
        rospy.loginfo("🔥 开始书写电子系徽图案...")
        
        # 为了让墨水有时间晕染，并且防止底层规划器拥堵，设定笔画间距
        delay = 0.3

        # ==========================================
        # 第一层：金色主框架
        # ==========================================
        # 1. 顶部金色小横杠 (居中，长 6cm)
        self.draw_heng(offset_x=0.05, offset_y=-0.03, length=0.06)
        rospy.sleep(delay)

        

        # ==========================================
        # 第二层：上半部分蓝色“雨”字头变形
        # ==========================================
        # 左侧竖线下滴
        self.draw_shu(offset_x=0.03, offset_y=0.03, length=0.025)
        # 左侧连接主轴的小横杠 (从中心偏左起笔，向左画)
        self.draw_heng(offset_x=0.03, offset_y=0.01, length=0.02)
        rospy.sleep(delay)

        # 右侧竖线下滴
        self.draw_shu(offset_x=0.03, offset_y=-0.03, length=0.025)
        # 右侧连接主轴的小横杠 (从最右侧起笔，向中心画)
        self.draw_heng(offset_x=0.03, offset_y=-0.03, length=0.02)
        rospy.sleep(delay)

        # 【重点修复】：内部四点 (雨字头的四滴水)
        # 上面两滴
        self.draw_dian(offset_x=0.03, offset_y=0.012)  # 左上点
        self.draw_dian(offset_x=0.03, offset_y=-0.021) # 右上点
        # 下面两滴
        self.draw_dian(offset_x=0.02, offset_y=0.012)  # 左下点
        self.draw_dian(offset_x=0.02, offset_y=-0.021) # 右下点
        rospy.sleep(delay)

        # ==========================================
        # 第三层：下半部分蓝色梳齿状结构 (对称的‘E’和反‘E’)
        # ==========================================
        # --- 左侧梳齿 ---
        self.draw_shu(offset_x=-0.01, offset_y=0.03, length=0.04)   # 左外围竖线
        self.draw_heng(offset_x=-0.01, offset_y=0.01, length=0.02)  # 上横
        self.draw_heng(offset_x=-0.03, offset_y=0.01, length=0.02)  # 中横
        self.draw_heng(offset_x=-0.05, offset_y=0.01, length=0.02)  # 下横
        rospy.sleep(delay)

        # --- 右侧梳齿 ---
        self.draw_shu(offset_x=-0.01, offset_y=-0.03, length=0.04)  # 右外围竖线
        self.draw_heng(offset_x=-0.01, offset_y=-0.03, length=0.02) # 上横
        self.draw_heng(offset_x=-0.03, offset_y=-0.03, length=0.02) # 中横
        self.draw_heng(offset_x=-0.05, offset_y=-0.03, length=0.02) # 下横

        # ==========================================
        # 第四部分：压轴大笔 —— 中间金色主干（竖弯钩）
        # ==========================================
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
        # 【核心修正】：加入桌面倾斜误差补偿！
        # 既然跑到这里笔尖够不到纸了，我们就让它比标准高度再往下压 3 毫米 (-0.003)
        # 你可以根据实际情况调整这个值，比如 -0.002 或 -0.004
        wpose.position.z = self.Z_PRESS_NORM - 0.003 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= 0.04  # 向右出钩
        wpose.position.z = self.Z_PRESS_LIGHT 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += 0.01  # 往回踢一点点出锋
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = self.Z_HOVER
        waypoints.append(copy.deepcopy(wpose))

        self.execute_cartesian_path(waypoints)
        
        rospy.loginfo("🎉 系徽图案绘制完成！")
        

if __name__ == '__main__':
    try:
        robot = KinovaCalligraphy()
        rospy.sleep(1) 
        robot.move_to_start_position()
        # 现在的坐标系变成了“相对偏移量”(Offset)。
        # (0, 0) 就是你运行脚本时机械臂所在的位置！
        # 我们让四个笔画在这个中心点周围分散开来写。
        
        # robot.draw_heng(0.0, -0.05)
        # rospy.sleep(0.5)
        
        # robot.draw_shu(0.0, 0.05)
        # rospy.sleep(0.5)
        
        # robot.draw_pie(0.05, 0.0)
        # rospy.sleep(0.5)
        
        # robot.draw_dian(0.05, -0.05)
        
        # robot.write_shi()

        robot.write_dept_logo()
        # robot.print_current_pose()
        
        rospy.loginfo("所有单笔画测试完成！")

    except rospy.ROSInterruptException:
        pass
