# WPR系列机器人ROS2仿真工具

## 介绍课程
Bilibili: [机器人操作系统 ROS2 入门教材](https://www.bilibili.com/video/BV1oz421v7tB)  
Youtube: [机器人操作系统 ROS2 入门教材](https://www.youtube.com/watch?v=j0foOvBqQTc)

## 配套教材书籍
《机器人操作系统（ROS2）入门与实践》  
![视频课程](./media/book_1.jpg)
淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)

## 系统版本

- ROS2 Jazzy (Ubuntu 24.04)
- ROS2 Humble (Ubuntu 22.04，历史兼容路线)

## 兼容性说明

- 当前默认路线是 Jazzy，仿真栈使用 `gz_sim` / `ros_gz` / `gz_ros2_control`。
- Humble 路线继续使用 Gazebo Classic，仅作为课程旧环境参考。
- Jazzy 已完成主场景、SLAM、Nav2、机械臂仿真和送水实验入口的构建适配；部分旧模型文件仍保留 Classic 写法，主要作为历史资源。

## 使用说明

下面以 ROS2 Jazzy + Ubuntu 24.04 为默认环境。所有命令默认在同一个 ROS2 工作区内执行，例如 `~/ros2_ws`。

### 1. 安装与编译

获取源码:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/huangyuya520/wpr_simulation2.git
```

安装 Jazzy 依赖:

```bash
cd ~/ros2_ws/src/wpr_simulation2
chmod +x scripts/install_for_jazzy.sh
./scripts/install_for_jazzy.sh
```

编译并加载工作区:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select wpr_simulation2
source install/setup.bash
```

每次打开新终端运行本包功能前，都需要重新执行:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

当前默认构建策略:

- 默认编译 Jazzy 常用仿真入口所需节点。
- `demo_cpp/` 课程示例节点默认不编译，减少首次构建时间。
- `BUILD_TESTING` 默认关闭，避免普通使用时额外配置 lint/test 目标。

需要课程配套 demo 节点时再显式开启:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DWPR_BUILD_TUTORIAL_DEMOS=ON
source install/setup.bash
```

常用构建开关:

| 场景 | 命令 |
|---|---|
| 只编译本包 | `colcon build --symlink-install --packages-select wpr_simulation2` |
| 开启课程 demo 节点 | `colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DWPR_BUILD_TUTORIAL_DEMOS=ON` |
| 关闭点云工具节点 | `colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DWPR_BUILD_POINTCLOUD_TOOLS=OFF` |
| 关闭运行资源安装 | `colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DWPR_INSTALL_RUNTIME_ASSETS=OFF` |
| 开启测试和 lint | `colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DBUILD_TESTING=ON` |

### 2. 快速启动

最小可运行场景:

```bash
ros2 launch wpr_simulation2 wpb_simple.launch.py
```

![wpb_simple pic](./media/wpb_simple.png)

如果 Gazebo 或 RViz 已经打开，建议先关闭旧进程，再重新启动新场景，避免 `/clock`、TF 或控制器状态残留。

### 3. 功能入口速查

| 功能 | 启动命令 | 说明 |
|---|---|---|
| 简单空场景 | `ros2 launch wpr_simulation2 wpb_simple.launch.py` | 适合首次验证 Gazebo、机器人模型、传感器和运动控制 |
| 机械臂机器人 | `ros2 launch wpr_simulation2 wpb_mani.launch.py` | 启动带机械臂的 WPB Home 机器人 |
| RoboCup Home 场景 | `ros2 launch wpr_simulation2 robocup_home.launch.py` | 启动家庭环境场景 |
| RoboCup Home + 机械臂 | `ros2 launch wpr_simulation2 robocup_home_mani.launch.py` | 适合抓取、送水等综合实验前置场景 |
| 彩球场景 | `ros2 launch wpr_simulation2 wpb_balls.launch.py` | 适合颜色、目标跟随和视觉实验 |
| 人脸场景 | `ros2 launch wpr_simulation2 wpb_face.launch.py` | 适合摄像头和人脸检测相关实验 |
| 物体识别场景 | `ros2 launch wpr_simulation2 wpb_objects.launch.py` | 适合点云聚类、物体识别实验 |
| 桌面场景 | `ros2 launch wpr_simulation2 wpb_table.launch.py` | 适合桌面物体、机械臂交互实验 |
| SLAM 建图 | `ros2 launch wpr_simulation2 slam.launch.py` | 启动 Gazebo、SLAM Toolbox 和 RViz |
| Nav2 导航 | `ros2 launch wpr_simulation2 navigation.launch.py` | 加载地图并启动导航流程 |

### 4. 遥控机器人

本包内置键盘控制节点，推荐优先使用:

```bash
ros2 run wpr_simulation2 keyboard_vel_cmd
```

也可以使用系统通用键盘遥控:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用检查命令:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /cmd_vel --once
ros2 run tf2_tools view_frames
```

### 5. SLAM 建图与保存地图

启动 SLAM:

```bash
ros2 launch wpr_simulation2 slam.launch.py
```

新开一个终端，遥控机器人移动建图:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run wpr_simulation2 keyboard_vel_cmd
```

![wpb_gmapping pic](./media/wpb_gmapping.png)

保存地图前，确认 `/map` 已经发布，并且 RViz 中 `Map` 显示正常:

```bash
ros2 topic echo /map --once
```

保存地图:

```bash
cd ~/ros2_ws/src/wpr_simulation2/maps
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

执行成功后会生成:

- `map.yaml`
- `map.pgm`

注意包名必须写成 `nav2_map_server`，不能写成 `nav2 map server`。

### 6. Nav2 导航

确认已经编译并 source 工作区后启动导航:

```bash
ros2 launch wpr_simulation2 navigation.launch.py
```

![wpb_navigation pic](./media/wpb_navigation.png)

在 RViz 中按顺序操作:

1. 使用 `2D Pose Estimate` 设置机器人初始位姿。
2. 使用 `Nav2 Goal` 或 `2D Goal Pose` 发送目标点。
3. 观察路径规划、局部避障和机器人移动状态。

如果机器人不移动，优先检查:

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /amcl_pose --once
ros2 action list | grep navigate_to_pose
```

### 7. Jazzy 兼容性与已知限制

- 当前默认路线是 ROS2 Jazzy，仿真使用 `gz_sim`、`ros_gz` 和 `gz_ros2_control`。
- Humble / Gazebo Classic 入口只作为课程旧环境参考，推荐新实验优先使用 Jazzy 入口。
- 原 Gazebo Classic 抓取修正类插件还没有完整迁移为 Gazebo Sim 系统插件。
- `wpv3.model`、`wpr1.model` 等早期历史模型仍可能包含 `libgazebo_ros_*` Classic 插件引用，直接运行时可能报错。
- `map_tools.launch.py`、`navigation.launch.py`、`wpb_scene_1.launch.py` 依赖 Nav2 相关组件。课程旧版 `wp_map_tools` 在 Jazzy apt 源中不可用；送水实验建议使用 `home_pkg` 内置的 `waypoint_navi_server.py`。

### 8. 常见问题

`Package 'wpr_simulation2' not found`

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Gazebo 能启动但模型/资源缺失:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DWPR_INSTALL_RUNTIME_ASSETS=ON
source install/setup.bash
```

地图保存失败:

- 确认已经安装 `ros-jazzy-nav2-map-server`。
- 确认 `/map` 有数据。
- 让机器人继续移动几秒，等待 SLAM 完成当前地图更新。

机器人不响应键盘控制:

- 确认键盘控制终端处于当前焦点。
- 确认 `/cmd_vel` 有速度消息。
- 确认 Gazebo 场景中的控制器已经启动完成。
