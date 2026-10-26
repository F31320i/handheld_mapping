# handheld_mapping

手持建图装置：**Livox Mid-360 + Logitech C310**，跑 FAST-LIVO2 做里程计，自己写的
位姿图后端做回环和出图。建图精度优先，其次才是速度。

一套手持走位走完，交付三份东西：优化后的彩色点云（PCD）、关键帧轨迹（TUM）、
通过检验的回环边（CSV）。

```
handheld_mapping/     手持项目的本体：launch / config / 标定脚本 / 两个自研节点
                      （livox_livo2_adapter、v4l2_capture）。用法看它的 README
vector_mapping/       位姿图后端 pgo_backend_node：关键帧、回环 GICP 校验、
                      GTSAM 图优化、增量地图、PCD/TUM/CSV 输出
reference/            不属于任何包：livo2_odom_bridge 的留档副本（见其中的 README）
```

## 依赖

本仓库只含**自己写的**两个包。整条链还需要下面这些，都不在这里：

| 包 | 来源 / 说明 |
|---|---|
| `livox_ros_driver2` | Livox 官方 ROS 2 驱动（ROS2 分支），另需 `Livox-SDK2` |
| `fast_livo` | FAST-LIVO2 的 ROS 2 版：`https://github.com/U-AMC/FAST-LIVO2-ROS2`；另需 `rpg_vikit`：`https://github.com/U-AMC/rpg_vikit_rational_polynomial` |
| `vector_localization` | 提供 `livo2_odom_bridge`（把 LIVO2 的 IMU 位姿换成 `base_link` 位姿 + `odom->base_link` TF）。源码副本见 `reference/` |
| `fast_limo` | 只在「不接相机、纯雷达」的那条链上用，可不用 |
| `usb_cam` / `camera_calibration` | `sudo apt install ros-humble-usb-cam ros-humble-camera-calibration`（相机内参标定用；建图本身走自研的 `v4l2_capture`） |
| `GTSAM` | `vector_mapping` 的图优化；Ubuntu 用 `sudo apt install libgtsam-dev` 或自行编译 |
| PCL / Eigen | `vector_mapping` 的地图与配准 |
| `rmw_zenoh_cpp` + `rmw_zenohd` | 本机所有跑法都先起 Zenoh router，否则表现为「节点都在、但机器人不出现、lifecycle 死等」 |

## 编译

放进任意工作空间的 `src/` 下即可（两个包是并列的，colcon 会各找一个）：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_local
git clone https://github.com/F31320i/handheld_mapping.git src/handheld_mapping
colcon build --merge-install \
  --packages-select handheld_mapping vector_mapping \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

`vector_mapping` 也可以单独编进别的工程——它不依赖本仓库的任何东西，只吃
ROS 2 / PCL / GTSAM。

## 用法

完整流程（接线、相机内参、LiDAR-相机外参标定、起传感器、建图、存图、常见坑）都在
**[`handheld_mapping/README.md`](handheld_mapping/README.md)**，这里只给最短路径：

```bash
# 0. 先起 Zenoh router（另开一个终端，常驻）
ros2 run rmw_zenoh_cpp rmw_zenohd

# 1. 建图（相机 + 雷达 + LIVO2 + PGO 后端）
ros2 launch handheld_mapping mapping_livo2.py \
    mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json

# 2. 存图（Ctrl-C 也会自动存一份到 $HOME/pgo_map_<时间戳>.pcd）
ros2 service call /pgo/save_map std_srvs/srv/Trigger
```

## 说明

- `config/MID360_config.json` 里是**厂商示例 IP**（主机 `192.168.1.50`、雷达
  `192.168.1.160`），和本机无关；实际用的是 `~/.config/handheld_mapping/`
  下由 `scripts/set_mid360_ips.py` 生成的那份。
- 本仓库是个人手持架的工作记录，`vector_mapping/package.xml` 的 license 字段还是
  占位的 `TODO`。
