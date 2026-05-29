import rospy
from geometry_msgs.msg import Point

# 假设你的机械臂库是这样控制的
# import your_robot_arm_library as arm

def callback(data):
    rospy.loginfo(f"接收到坐标: X={data.x}, Y={data.y}")
    
    # 【坐标转换核心】：这里需要把你刚才的像素坐标转换为机械臂的空间坐标
    # 比如：x_arm = (data.x - 320) * 比例系数
    # y_arm = (data.y - 240) * 比例系数
    
    # 【动作执行】：调用机械臂库
    # arm.move_to(x_arm, y_arm, z_height=10) # 移动到墨盘上方
    # arm.dip_ink()                         # 执行蘸水动作
    # arm.move_to_paper()                   # 回到纸张位置

def listener():
    rospy.init_node('robot_controller', anonymous=True)
    # 订阅你的 AI 发布的坐标话题
    rospy.Subscriber('/ink_tray_position', Point, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()