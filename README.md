# EE368-Project

本项目是一个基于 ROS、MoveIt 和 Kinova Gen3 Lite 机械臂的机器人书法项目。主要目标是让机械臂能够根据输入文字自动生成汉字笔画轨迹，并控制毛笔完成书写；同时通过视觉检测实现毛笔自动蘸墨水。

## 项目功能

- 使用 `all2.py` 实现任意汉字书写：程序读取 `hanzi-writer-data` 中的汉字笔画数据，将笔画中心线转换为机械臂末端的笛卡尔轨迹，并通过 MoveIt 控制机械臂书写。
- 使用 `test3.py` 实现毛笔蘸墨水：程序结合摄像头、OpenCV 和 YOLO 模型检测墨水位置，引导机械臂移动到墨水区域完成蘸墨动作。
- 支持基础笔画测试、图像路径转换、手眼标定等辅助流程。

## 目录结构

```text
catkin_ws/
├── src/
│   ├── calligraphy_project/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── scripts/
│   │       ├── all2.py                  # 任意汉字书写主程序
│   │       ├── test3.py                 # 毛笔蘸墨水程序
│   │       ├── project4.py              # 图像/文字路径转换辅助脚本
│   │       ├── kinova_calligraphy.py    # 基础笔画和机械臂书写测试
│   │       ├── best.pt                  # YOLO 检测模型
│   │       ├── handeye_calibrate.launch # 手眼标定启动文件
│   │       └── package.json             # hanzi-writer-data 依赖
│   └── easy_handeye/                    # 手眼标定相关 ROS 包
├── .gitignore
└── README.md
```

## 环境依赖

建议环境：

- Ubuntu + ROS 1，推荐 ROS Noetic
- Python 3
- catkin
- MoveIt
- Kinova Gen3 Lite ROS/MoveIt 环境
- OpenCV
- Ultralytics YOLO，用于 `test3.py` 的视觉检测
- Node.js 和 npm，用于安装汉字笔画数据 `hanzi-writer-data`

ROS 依赖主要包括：

- `rospy`
- `geometry_msgs`
- `moveit_commander`
- `tf2_ros`
- `std_msgs`
- `std_srvs`

## 安装与编译

克隆仓库：

```bash
git clone https://github.com/kkkkkkkkkkkkkkkkkyyyyyyyyyy/EE368-Project.git
cd EE368-Project
```

安装汉字笔画数据：

```bash
cd src/calligraphy_project/scripts
npm install
cd ../../..
```

编译 catkin 工作空间：

```bash
catkin_make
source devel/setup.bash
```

## 使用方法

运行前请先启动 Kinova 机械臂和对应的 MoveIt 控制环境，并确认机械臂、相机、纸面和墨水位置已经完成标定。

### 1. 任意汉字书写

`all2.py` 是本项目的主要书写程序。它会读取汉字笔画数据，根据当前机械臂末端姿态建立书写坐标系，然后控制机械臂按笔顺完成书写。

运行：

```bash
source devel/setup.bash
rosrun calligraphy_project all2.py
```

如果需要修改书写内容、字体大小、字间距或采样密度，可以在 `src/calligraphy_project/scripts/all2.py` 中调整对应参数。

需要重点检查的机械臂配置：

```python
self.group_name = "arm"
robot_description = "my_gen3_lite/robot_description"
ns = "my_gen3_lite"
```

如果你的 Kinova 命名空间、MoveIt group 或 robot description 不同，需要按实际环境修改。

### 2. 毛笔蘸墨水

`test3.py` 用于实现毛笔自动蘸墨水。程序会调用摄像头画面和 YOLO 模型 `best.pt`，识别墨水区域后发布或执行对应的机械臂移动逻辑。

运行：

```bash
source devel/setup.bash
cd src/calligraphy_project/scripts
python3 test3.py
```

使用前需要确认：

- 摄像头编号正确，例如 `cv2.VideoCapture(...)` 中的索引。
- `best.pt` 模型文件存在于脚本目录下。
- 相机坐标到机械臂坐标的映射关系已经根据实际环境调好。
- 蘸墨高度、下降深度和安全悬停高度已经测试过。

### 3. 手眼标定

如果需要重新标定相机和机械臂关系，可以运行：

```bash
source devel/setup.bash
roslaunch calligraphy_project handeye_calibrate.launch
```

## 注意事项

- 首次运行前一定要先进行空写测试，确认轨迹不会碰撞桌面、纸张边缘或夹具。
- `all2.py` 中的 Z 轴高度参数会直接影响毛笔是否接触纸面，修改后需要低速测试。
- `test3.py` 中的相机编号、检测模型和坐标转换参数通常需要根据现场设备重新调整。
- 机械臂执行过程中请保持急停按钮可用。
- `build/`、`devel/`、`node_modules/` 等生成文件不会上传到 GitHub；`node_modules/` 可通过 `npm install` 重新生成。
