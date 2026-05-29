# EE368-Project

This repository is a ROS catkin workspace for a Kinova robotic-arm calligraphy project. It contains motion-control scripts, image/YOLO-based detection experiments, Hanzi stroke-path generation, and hand-eye calibration support.

## Project Structure

```text
catkin_ws/
├── src/
│   ├── calligraphy_project/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── scripts/
│   │       ├── all2.py
│   │       ├── project4.py
│   │       ├── kinova_calligraphy.py
│   │       ├── robot_controller.py
│   │       ├── handeye_calibrate.launch
│   │       ├── best.pt
│   │       └── package.json
│   └── easy_handeye/
├── .gitignore
└── README.md
```

## Main Components

- `calligraphy_project`: Main ROS package for the robotic-arm calligraphy workflow.
- `scripts/all2.py`: Writes Chinese characters by reading stroke medians from `hanzi-writer-data` and converting them into robot Cartesian paths.
- `scripts/project4.py`: Converts image/text paths into robot path points.
- `scripts/kinova_calligraphy.py`: Basic Kinova stroke primitives and motion tests.
- `scripts/1.py`, `scripts/test3.py`: YOLO/OpenCV experiments for ink-tray or visual detection.
- `scripts/handeye_calibrate.launch`: Launch file for hand-eye calibration.
- `easy_handeye`: Included ROS hand-eye calibration dependency packages.

## Requirements

Recommended environment:

- Ubuntu with ROS Noetic or another compatible ROS 1 distribution
- Python 3
- catkin
- MoveIt
- Kinova Gen3 Lite ROS/MoveIt configuration available in the ROS environment
- OpenCV
- Ultralytics YOLO, if using the detection scripts
- Node.js and npm, only for restoring Hanzi stroke data

ROS package dependencies used by `calligraphy_project` include:

- `rospy`
- `geometry_msgs`
- `moveit_commander`

The included `easy_handeye` packages also require common ROS packages such as `tf2_ros`, `std_msgs`, `std_srvs`, and OpenCV bindings.

## Setup

Clone the repository:

```bash
git clone https://github.com/kkkkkkkkkkkkkkkkkyyyyyyyyyy/EE368-Project.git
cd EE368-Project
```

Restore the Hanzi stroke data used by `all2.py`:

```bash
cd src/calligraphy_project/scripts
npm install
cd ../../..
```

Build the catkin workspace:

```bash
catkin_make
source devel/setup.bash
```

If your ROS environment uses a different Kinova namespace, MoveIt group, or robot description name, update the values in `src/calligraphy_project/scripts/all2.py` before running:

```python
self.group_name = "arm"
robot_description = "my_gen3_lite/robot_description"
ns = "my_gen3_lite"
```

## Usage

Start the robot and MoveIt launch files required for your Kinova arm first. Then source the workspace:

```bash
source devel/setup.bash
```

Run a basic calligraphy script:

```bash
rosrun calligraphy_project kinova_calligraphy.py
```

Run the Hanzi stroke-data writing workflow:

```bash
rosrun calligraphy_project all2.py
```

Run hand-eye calibration launch file:

```bash
roslaunch calligraphy_project handeye_calibrate.launch
```

Run the YOLO detection experiment:

```bash
cd src/calligraphy_project/scripts
python3 1.py
```

## Safety Notes

- Verify all workspace coordinates and Z heights before allowing the robot to touch paper.
- Test motions in the air before writing with the pen.
- Keep the emergency stop available when running MoveIt execution scripts.
- Camera index, robot namespace, and calibration values may need to be changed for a different machine.

## Git Notes

Generated catkin folders such as `build/`, `devel/`, and `logs/` are intentionally ignored. The large `node_modules/` folder is also ignored; restore it with `npm install` from `src/calligraphy_project/scripts`.
