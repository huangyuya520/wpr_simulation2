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

### 一、 启智ROS机器人
1. 获取源码:
```
cd ~/ros2_ws/src/
git clone https://github.com/huangyuya520/wpr_simulation2_changestoUbuntu24.04LTSversions.git
```
2. 安装依赖项:  
ROS2 Jazzy (Ubuntu 24.04)
```
cd /home/huangyuya/Desktop/ROSExperimenteight/src/wpr_simulation2/scripts
./install_for_jazzy.sh
```
3. 编译
```
cd /home/huangyuya/Desktop/ROSExperimenteight
colcon build --symlink-install
```

当前默认构建策略:
- 默认只编译 Jazzy 常用主场景所需节点，减少首次构建和增量构建时间。
- `demo_cpp/` 下的课程示例节点默认不参与编译。
- `BUILD_TESTING` 默认关闭，避免本地构建时额外配置 lint/test 目标。

如果你需要课程配套 demo 节点，再显式打开它们:
```
cd /home/huangyuya/Desktop/ROSExperimenteight
colcon build --symlink-install --cmake-args -DWPR_BUILD_TUTORIAL_DEMOS=ON
```

如果你只是反复修改这个包，建议这样编译，会更快一些:
```
cd /home/huangyuya/Desktop/ROSExperimenteight
colcon build --symlink-install --packages-select wpr_simulation2
```

如果你只跑简单场景，还想进一步缩短构建时间，可以关闭点云节点或运行资源安装:
```
cd /home/huangyuya/Desktop/ROSExperimenteight
colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DWPR_BUILD_POINTCLOUD_TOOLS=OFF
colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DWPR_INSTALL_RUNTIME_ASSETS=OFF
```

需要重新打开测试和 lint 时:
```
cd /home/huangyuya/Desktop/ROSExperimenteight
colcon build --symlink-install --packages-select wpr_simulation2 --cmake-args -DBUILD_TESTING=ON
```

简单场景:
```
ros2 launch wpr_simulation2 wpb_simple.launch.py 
```
![wpb_simple pic](./media/wpb_simple.png)

Jazzy 下当前已迁移的常用入口:
```
ros2 launch wpr_simulation2 wpb_simple.launch.py
ros2 launch wpr_simulation2 wpb_mani.launch.py
ros2 launch wpr_simulation2 robocup_home.launch.py
ros2 launch wpr_simulation2 robocup_home_mani.launch.py
ros2 launch wpr_simulation2 wpb_balls.launch.py
ros2 launch wpr_simulation2 wpb_face.launch.py
ros2 launch wpr_simulation2 wpb_objects.launch.py
ros2 launch wpr_simulation2 wpb_table.launch.py
```

Jazzy 当前限制:
- 原 Gazebo Classic 抓取修正类插件还没有完整迁移到新 Gazebo 系统插件。
- `wpv3.model`、`wpr1.model` 等更早期的历史模型文件还没有迁移到 `gz_sim`，如果直接使用它们，仍可能遇到 `libgazebo_ros_*` 相关错误。
- `map_tools.launch.py`、`navigation.launch.py`、`wpb_scene_1.launch.py` 还依赖 `nav2_bringup`。课程旧版 `wp_map_tools` 在 Jazzy apt 源中不可用；送水实验请使用 `home_pkg` 内置的 `waypoint_navi_server.py`。

SLAM环境地图创建:
```
ros2 launch wpr_simulation2 slam.launch.py 
ros2 run wpr_simulation2 keyboard_vel_cmd
```
![wpb_gmapping pic](./media/wpb_gmapping.png)

保存地图前，请确认系统已经安装 `nav2_map_server`:
```
sudo apt-get install ros-jazzy-nav2-map-server
```

地图保存命令里的包名必须写成带下划线的 `nav2_map_server`，不能写成带空格的 `nav2 map server`:
```
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

执行成功后，当前目录下会生成 `map.yaml` 和 `map.pgm` 两个文件。
如果仍然失败，先确认 RViz 中 `Map` 已经不再是 `Warn`，或者先让机器人继续移动几秒，确保 `/map` 已经真正发布出来。

如果本机没有安装 `rqt_robot_steering`，优先使用工作区自带的键盘控制节点:
```
source install/setup.bash
ros2 run wpr_simulation2 keyboard_vel_cmd
```

也可以使用系统里常见的备用控制节点:
```
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Navigation导航:
```
ros2 launch wpr_simulation2 navigation.launch.py 
```
![wpb_navigation pic](./media/wpb_navigation.png)
