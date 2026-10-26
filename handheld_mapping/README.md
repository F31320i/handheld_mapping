# handheld_mapping

手持建图：Livox Mid-360 + Logitech C310 + FAST-LIVO2 + 本项目自研位姿图后端。
本包只有 launch / 配置 / 脚本，外加两个**必须自己写**的编译节点（雷达适配、相机采集，
见第 6 节），重活在复用的包里：`livox_ros_driver2`、`fast_livo`、
`vector_localization`(livo2_odom_bridge)、`vector_mapping`(pgo_backend / map_accumulator)。
`usb_cam` 只留给标定用。

数据流（见第 6 节，为什么中间要有一个适配节点）：

```
/livox/lidar (CustomMsg) --livox_livo2_adapter--> /livox/lidar_livo (PointCloud2) --+
/livox/imu   (Imu)       --livox_livo2_adapter--> /livox/imu_sync                 --+--> FAST-LIVO2
/image_raw   v4l2_capture (主机时钟)  ---------------------------------------------- +

FAST-LIVO2 -> /aft_mapped_to_init -> livo2_odom_bridge -> /odometry/lio + TF
           -> /cloud_registered (camera_init, 已上色)
/cloud_registered + /odometry/lio -> pgo_backend -> /pgo/map, /pgo/trajectory,
    /pgo/loops, map->odom TF
/cloud_registered -> map_accumulator -> /handheld/map_raw（无回环版本，用来对照）
```

---

## 0. 环境（已在本机装好）

| 项 | 状态 |
|---|---|
| `ros-humble-usb-cam` `camera-calibration` `image-pipeline` `rqt-image-view` `rosbag2` | apt 已装 |
| `v4l-utils`（`v4l2-ctl`） | apt 已装 |
| Livox-SDK2 | 源码在 `src/vector_robot/submodules/Livox-SDK2`，同时装到了 `/usr/local` |
| `livox_ros_driver2` | 已编入本工作空间，`install/lib/livox_ros_driver2/livox_ros_driver2_node` |

重编（livox_sdk2 是工作空间内的源码包，必须一起编，不能用 `--packages-select`）：

```bash
cd ~/ros2_ws_local
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-up-to livox_ros_driver2 \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release -Wno-dev
colcon build --merge-install --packages-select handheld_mapping \
    --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

---

## 1. 接通雷达（网络）

Mid-360 默认是静态 IP，所以电脑那口网卡要设成同网段。**实测这台雷达是
`192.168.1.109`**（MAC `58:b8:58:c0:7e:0f`），电脑这边定 `192.168.1.2`。

注意两个坑：

- 手工 `sudo ip addr add` 加的地址**不持久**。NetworkManager 一旦把 enp5s0 判成
  `disconnected`（插拔、VPN 切换、睡眠唤醒都会），它会把这地址一起清掉，
  表现就是「刚才还能收点，现在 ping 不通」。已经建了一个随接口自动激活的
  profile 顶着（只用 192.168.1.2/24、`never-default`，不会抢默认路由）：

  ```bash
  nmcli connection show handheld-lidar          # 看它还在不在
  sudo nmcli connection up handheld-lidar       # 需要时手动拉起
  ip -4 addr show enp5s0                        # 应该看到 192.168.1.2/24
  ping -c 3 192.168.1.109
  # 不想要了就删掉：
  # sudo nmcli connection delete handheld-lidar
  ```

  如果换了雷达或改了设备 IP，就删掉这个 profile 重建成对应网段。

- 开 VPN（Clash/mihomo TUN 之类）本身不冲突：策略路由只把默认流量导给 tun，
  具体路由仍走 main 表。但**接口地址丢失**才是真问题，见上。

`config/MID360_config.json` 是模板（主机 `192.168.1.50`、雷达 `192.168.1.160`），
实机不要直接用模板，生成一份自己的：

```bash
# 不带参数时会把本机所有 IPv4 列出来，照着自己填
ros2 run handheld_mapping set_mid360_ips.py
# 实机这台：
ros2 run handheld_mapping set_mid360_ips.py \
    --host-ip 192.168.1.2 --lidar-ip 192.168.1.109
# 生成在 ~/.config/handheld_mapping/MID360_config.json
```

雷达 IP 不在自己网段时只能靠 Livox 官方工具改，`set_mid360_ips.py` 只是写好配置，
不会去改设备。驱动要占用 UDP 56100–56500（配置里 host 端口是基准 +100）；如果收不到
点，先看防火墙 `sudo ufw status`，再看是不是上一份驱动的进程没退干净。

> 之后所有 launch 都加 `mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json`。

## 2. 相机

C310（046d:081b）是**定焦**、无光学/数码变焦，V4L2 里没有 focus/zoom 控件，正常。
跑 **640x480 @ 30 Hz YUYV**：一次测量里 YUYV 的固定延迟是 0.47 s、MJPG→rgb8 是
0.96 s（另一次测量 YUYV 也有 0.95 s，所以两个数的绝对值别太当真）；
关键是这个延迟是常数，LIVO2 会等属于该扫描的图像。`cv_bridge` 能把
`yuv422_yuy2` 直接转 `bgr8`，不需要额外的转换节点。

```bash
v4l2-ctl --list-devices                     # 确认 /dev/video0
ros2 run handheld_mapping set_camera_controls.sh            # 默认：手动曝光 28.5 ms + 亮度 120
ros2 run handheld_mapping set_camera_controls.sh /dev/video0 auto
```

脚本会固定 `power_line_frequency=50`（国内 50 Hz 灯光，否则会滚条纹）。
手动曝光在室内固定光照下更稳；亮度变化大的场地就用 auto。
标定和建图请用**同一套**曝光设置。

**太暗时先看 `brightness`，不是曝光。** 两者是独立的：曝光是积分时间（30 fps 的上限
是 33.3 ms = 333），`brightness` 是相机内的数字偏移，只抬黑位、不增加运动模糊。
这台 C310 曾被写进 `brightness=50`（usb_cam 的默认值，驱动默认是 128），于是曝光
拉到头画面还是暗的。实测（曝光固定 285）：

| `brightness` | 亮度均值 / 中位数 | 结论 |
|---|---|---|
| 50 | 42 / 44 | 太暗 |
| 120 | 96 / 105 | 正常，0% 过曝 |

实测（本机）：不跑脚本时自动曝光会拉长曝光时间并让帧率掉到 **24 Hz**；跑脚本后回到
**29.7 Hz**。所以建图前先跑它。`exposure_time_absolute` 写 100 读回是 105，这是这台
相机的量化，正常。

## 2.5 必须先起 Zenoh router

本机 ROS 2 用的是 `rmw_zenoh_cpp`。**没有 router 时节点之间互相发现不了**——
`ros2 topic list` 看不到 launch 里的节点，`check_streams.py` 全部显示 NO MESSAGE，
很容易误判成驱动没起来。每开机一次起一个就行：

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## 3. 先验检查（每次都先跑一遍）

```bash
ros2 launch handheld_mapping sensors.py \
    mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json
# 另一个终端
ros2 run handheld_mapping check_streams.py --seconds 8
```

期望（本机实测）：`/livox/lidar_livo`≈10 Hz（19968 点/帧）、`/livox/imu_sync`≈200 Hz、
`/image_raw`≈30 Hz，并且 `lidar_livo`、`imu_sync`、`image_raw` 的 `stamp - host`
都是**一个小的负数**（本机：雷达 −0.15～−0.20 s，IMU −0.09 s，图像 **−0.036 s**；
换回 usb_cam 会变成 −0.1～−1.0 s，见 6.2b）。
若 `lidar_livo` 与 `image_raw` 相差成千上万秒，说明适配节点没生效或 IP 配错
（此时 FAST-LIVO2 会一直等，不出轨迹）。

想肉眼看点云是否真的打到相机视窗里，直接跳到第 4.2 节。

只想确认 IMU 好不好（把架子放桌上别动）：

```bash
ros2 run handheld_mapping watch_imu.py          # 每 0.25 s 刷新一屏，Ctrl-C 退出
ros2 run handheld_mapping watch_imu.py --topic /livox/imu_sync   # 看适配之后的
ros2 run handheld_mapping watch_imu.py --plain  # 不刷屏，一块一块横着输出（能进日志）
```

读数怎么看：静止时 `|a| ≈ 9.8`（z 轴为主，roll/pitch 接近 0）、**窗口标准差 σ(acc) 几个 mg、
σ(gyr) < 0.01 rad/s** 就是好 IMU；`|ω|` 不为 0 但 σ 很小，那是**陀螺零偏**（MEMS 正常，
LIVO2 用开局 0.25 s 静止窗口估掉它），不是坏件。频率应该稳在 **200 Hz**，最大间隔别超过
十几毫秒。

看原始点云（不需要 PGO/LIVO2，固定帧是雷达自身）：

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix handheld_mapping)/share/handheld_mapping/rviz/lidar_only.rviz
```

> `/livox/lidar` 是 CustomMsg，**rviz 画不了**，必须用 `/livox/lidar_livo`。

## 4. 标定

### 4.1 相机内参

先设好 V4L2 控件（不设的话自动曝光会把角点糊掉，暗光下帧率还会掉到 24 Hz）：

```bash
ros2 run handheld_mapping set_camera_controls.sh
```

然后：

```bash
ros2 launch handheld_mapping calibrate_camera.py size:=8x6 square:=0.020
```

- **参数是 `名字:=值`，不是 `--size`。** `ros2 launch` 只认这种写法，`--size 8x6`
  会直接 argparse 报 `unrecognized arguments`。
- `size` 是**内角点**数：`docs/calib_boards_*.pdf` 第 6 页那张是 9×7 格，所以是 `8x6`。
- `square` 用米，而且**必须和实际印出来的格子量出来的值一致**（那张纸是 0.020）。
  填错不会报错，只是 fx/fy 按同样的比例缩放——LIVO2 会把每个点投成错误的大小。
- 在 **640x480**（建图实际跑的分辨率）下标定，别换分辨率。
- 让 X / Y / Size / Skew 全绿再按 SAVE。
- 结果在 `/tmp/calibrationdata.tar.gz` 里的 `ost.yaml`（路径写死在
  `camera_calibration` 里，改不了；`/tmp` 重启会清，标完马上拿走）：

**CALIBRATE 是灰的 / 彩虹圈时有时无**——两个都是上游的判据，不是相机的问题：

- `CALIBRATE` 只有两个条件之一满足才亮（`calibrator.py:487`）：库里攒够
  **40 个样本**，或者四个进度条都满。四根条是**跨度**，不是数量：
  X 跨度 ≥ 0.7、Y 跨度 ≥ 0.7、Size 跨度 ≥ 0.4、Skew 跨度 ≥ 0.5。
  所以要么把板子**从左边缘扫到右边缘、上边缘扫到下边缘**（全程完整可见），
  从占画面 1/4 走近到占 2/3，并且至少拍一次大角度倾斜。
- **同一个姿势重复拍不涨数**：新样本要跟库里每个样本的四参数 L1 距离 > 0.2
  才收（`calibrator.py:443`），否则静默丢弃——**屏幕上的圈照样画**。
  所以"画了圈但没进展"是正常的，看终端有没有 `*** Added sample N` 才准。
- 彩虹圈闪：默认带 `cv2.CALIB_CB_FAST_CHECK`，它会提前判定"这帧没有板"直接跳过。
  launch 里已经默认加了 `--disable_calib_cb_fast_check`（每帧全量检测，
  标定阶段这点 CPU 无所谓）；实在想退回就 `disable_fast_check:=false`。
- 板子别太远：20 mm 格子，1.5 m 外每格只有几像素，检测会时成时不成。
  站 0.4–1.0 m。

```bash
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp ost.yaml
# 先看一眼 fx/fy 是不是几百的量级、cx/cy 是不是靠近 320/240
ros2 run handheld_mapping camera_calib_to_livo2.py \
    --calib /tmp/ost.yaml --out src/handheld_mapping/config/camera_c310.yaml
```

改**源文件**再重编（`colcon build --packages-select handheld_mapping ...`）最干净；
直接写 install 里的那份也能跑，但下次重编会被覆盖。

**2026-10-26 第一次实测**（92 张样本，产物存在项目目录
`docs/camera_calib_2026-10-26/`，含 tar + ost.yaml）：

```
fx 828.60  fy 826.72   cx 315.18  cy 234.02   640x480
```

单张重投影误差中位数 0.35 px，说明模型和数据是自洽的。**但这组姿态对 fx 只有约
1.5% 的约束**：同样的角点换个初值重拟合会得到 fx 816.5，而全局 RMS 一模一样
（2.325 px）——焦距和 k2 在互抵。原因是板子拍得小（画面里中位只有 203 px 宽，
92 张里有 21 张不到 160 px，那几张的单张误差 5–7 px）。想更紧就**把板子拿近，
让棋盘占画面宽度一半以上**，并多拍大倾角。

**2026-10-26 第二轮实测（采用这一组）**（61 张样本，
`docs/camera_calib_2026-10-26_round2/`）：

```
fx 837.79  fy 837.21   cx 319.24  cy 232.25   640x480
D = [0.068842, 0.354198, -0.005487, -0.001138, 0]
```

这一轮板子拍得近（画面里中位 345 px 宽，61 张里只有 3 张不到 250 px），fx 因此
真的被约束住了：拿同样的角点重拟合得 fx 837.99，**差 0.2 px（0.03%）**，而第一轮
那组差 11 px。fx/fy 也自己对齐到了 0.07%。所以第一轮的 828.60 是被 k2 带偏的结果，
以 **837.79** 为准。

第一轮的 92 张没有作废，可以交叉验证：用第二轮的参数去算第一轮那批图的全局 RMS
是 2.170 px，反过来第一轮参数算自己 2.325 px、算第二轮 2.170 px——RMS 在这个量级
上区分不开对错（都只差 0.01 px），**能区分的是"重拟合会不会漂"**，这就是为什么
判断要看重拟合离散度而不是重投影误差。把两轮混在一起拟合得 fx 836.5，落在中间，
受第一轮小样本拖拽，不要用。

LIVO2 用裸 pinhole、**不做去畸变**，所以 D 只是个记录。这点残差影响多大可以算：
按上面这组 D，画面中心 0 px，右边 x=600 处 3.0 px，右下角 (620,440) 6.9 px。
对 640 宽的图来说可以接受；但要知道相机-雷达融合在画面最边缘会有这么多像素的系统
偏差，接 FAST-Calib 时如果边缘对不齐，先怀疑这个。

另外 `cameracalibrator` 写的 `ost.yaml` **没有 `%YAML:1.0` 头**，`cv2.FileStorage`
会直接拒（抛的还是 `SystemError` 而不是 `cv2.error`）。`camera_calib_to_livo2.py`
已改成 cv2 读不了就回落 PyYAML。
注意 `camera_c310.yaml` 必须是 pinhole：vikit 若走 RationalPolynomial 分支会读
全零畸变并 SIGFPE。LIVO2 本身**不使用**畸变参数，所以只要 fx/fy/cx/cy 对就行。

### 4.2 雷达-相机外参

```bash
ros2 launch handheld_mapping calibrate_lidar_camera.py \
    mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json
```

`project_points.py` 用和 FAST-LIVO2 完全相同的约定
`p_cam = Rcl * p_lidar + Pcl` 把当前帧投到 `/image_with_points` 上：

- `Pcl` = 相机光心在**雷达坐标系**（x 前 / y 左 / z 上）下的位置，尺子量，
  相机在雷达后面就是负 x。
- `Rcl` = 雷达轴到相机光学轴（x 右 / y 下 / z 前）的旋转，默认
  `[0,0,1, -1,0,0, 0,-1,0]`（相机朝雷达 +x，水平安装）。

```bash
ros2 param set /project_points Pcl "[0.02, 0.0, -0.03]"
ros2 param set /project_points Rcl "[0,0,1, -1,0,0, 0,-1,0]"
```

从几个不同距离/角度看，点贴上对应表面后把这两个数抄进
`config/livo2_mid360.yaml` 的 `extrin_calib.Rcl` / `Pcl`。

**不要动 `extrinsic_T` / `extrinsic_R`**：那是雷达在 IMU 坐标系里的位置
（出厂值取反，见配置里注释），和这里的相机外参是两件事。本项目把 base_link
定义成 IMU 机体，所以 `livo2_odom_bridge_handheld.yaml` 里的 `base_to_imu_*`
全是 0。

### 4.3 用 FAST-Calib 时内参填在哪

上游 `hku-mars/FAST-Calib` 是 ROS1 包，**没有装进本工作空间**；要用就单独克隆编译
（ROS2 可参考 `TommyBrownson/FAST-Calib_Ros2`、`MaroSalah02/FAST-Calib-ROS2`）。

它**必须**要内参：`include/common_lib.h` 的 `loadParameters()` 读 8 个平铺标量，
`src/qr_detect.hpp` 再拼成 `cameraMatrix = [[fx,0,cx],[0,fy,cy],[0,0,1]]`、
`distCoeffs = [k1,k2,p1,p2,0]`，拿去 `estimatePoseSingleMarkers` /
`estimatePoseBoard` / `projectPoints`。**没有 `camera_matrix` 或
`camera_intrinsic_path` 这种键**——那是别的 fork 的写法。

```bash
ros2 run handheld_mapping camera_calib_to_livo2.py \
    --calib docs/camera_calib_2026-10-26_round2/camera_c310_ost.yaml \
    --format fastcalib
```

输出直接贴进 `config/qr_params.yaml` 覆盖那 8 行：

```yaml
fx: 837.785540
fy: 837.208400
cx: 319.237410
cy: 232.249420
k1: 0.06884200
k2: 0.35419800
p1: -0.00548700
p2: -0.00113800
```

三个坑：

- **仓库自带的默认值属于 1280x960 的相机**（`fx 522 / cx 773 / cy 534`）。是替换，
  不是合并；留着就是静默算错。
- **没有 k3**。上游把第五个畸变系数写死成 0，所以我们的 `k3 = 0` 正好，别自己加。
- **上游不校验分辨率**。我们标在 640x480，喂给它的图也必须是 640x480。
  （FAST-Calib2 那个 ROS2 fork 加了这个校验，反而能救命。）

**站多远**：靶板 1400 × 1000 mm，fx ≈ 838、图 640×480，要让整块板进画面得
`838 × 1.4 / 640 ≈ 1.83 m`，高度约束是 1.74 m。也就是说 **1.5 m 根本拍不全**，
虽然 `qr_params.yaml` 的距离滤波默认 `x_max: 3.0` 看着更宽松。实际站
**2.0–2.5 m**：板宽 469–586 px、200 mm 的 ArUco 67–84 px，检测和滤波都舒服。
距离滤波默认值（`x 1.5–3.0`）不用改。

### 4.4 给 FAST-Calib 采数据

它吃两样东西：一个**装着雷达数据的 rosbag**，和**一张静态照片**（`image_path`，
它在这张图上检测 ArUco 并解出靶板位姿）。照片不是从 bag 里抽的，是单独给的文件。

三个终端：

```bash
# 终端 1 -- 传感器。mid360_config 必须传！包里那份是占位 IP（192.168.1.50/160），
# 传错的表现是驱动"Init lds lidar success"但一帧点云都不出。
source ~/ros2_ws_local/install/setup.bash
ros2 run handheld_mapping set_camera_controls.sh          # 曝光锁死，和标内参时一致
ros2 launch handheld_mapping sensors.py \
    mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json
```

```bash
# 终端 2 -- 存图。摆好板子、人退出去之后跑，它会抓下一帧。
source ~/ros2_ws_local/install/setup.bash
ros2 run handheld_mapping save_frame.py --out ~/calib/scene_a.png
```

```bash
# 终端 3 -- 录 10 秒。板子和人都不许动。
source ~/ros2_ws_local/install/setup.bash
ros2 bag record -o ~/calib/scene_a \
    /livox/lidar /livox/imu_sync /image_raw
# 数到 10 就 Ctrl-C
```

**每个终端都必须 source 工作空间的 `install/setup.bash`**：zenoh 的
`RMW_IMPLEMENTATION` 是 `vector_rmw_config` 这个包的 env hook 设的，只 source
`/opt/ros/humble/setup.bash` 的终端走的是默认 RMW，`ros2 topic list` 里什么都看不到
（而且不报错，就是空的）。另外 zenoh router 得在跑，见
[2.5 必须先起 Zenoh router](#25-必须先起-zenoh-router)。

10 秒的 bag 大约 **210 MB**，八成是图（640×480 的 yuv422 每帧 614 KB、30 Hz）。
不在乎的话可以把 `/image_raw` 去掉，反正是静态场景。

录的时候：

- **靶板静止**，架在三脚架/靠稳的地方。FAST-Calib 靠多帧累积出板面上的点，
  手拿着晃就是把它抹平。
- **人不要在 1.5–3.0 m 的窗口里**。那是距离滤波的范围，你的身体会被当成板面附近的
  杂点喂给平面拟合。
- 站 **2.0–2.5 m**（理由见 4.3），板子朝向雷达和相机中间，别偏到某一侧取景框外。
- **录三组，换位置不换距离**：正前方一组、雷达右前方一组、左前方一组。单视角对
  6 自由度的约束是弱的，上游为此专门给了 `multi_calib.launch`。既然建图精度优先，
  就录满三组。

录完先检查：

```bash
ros2 bag info ~/calib/scene_a
```

13.8 s 的实测参考值：`/livox/lidar` 138 条（10 Hz，类型
`livox_ros_driver2/msg/CustomMsg`）、`/image_raw` 411 条（30 Hz）、
`/livox/imu_sync` 2766 条（200 Hz）。**`/livox/lidar` 是 0 条就是 IP 传错了。**

- **别用 `ros2 run image_view image_saver` 存图**。Humble 上它接受 `filename`
  参数然后无视它，把 `left0000.jpg`、`left0001.jpg`……一路存到**当前目录**
  （实测 6 秒 175 张）。用 `save_frame.py`。
- 录的是**原生 `CustomMsg`**，不是 `/livox/lidar_livo`。后者是我们适配器转出来的
  PointCloud2，点上的 `timestamp` 是绝对纳秒，而上游 Livox 解析器把 `timestamp`
  当"扫描起始后的偏移秒"，喂过去时间轴会偏 1.7e9 秒。如果你的 ROS2 fork 只吃
  PointCloud2，那就改成录 `/livox/lidar_livo`——xyz 是对的，只是别指望时间戳。
- bag 是 **rosbag2（sqlite3）**，上游 FAST-Calib 是 ROS1 包。要么用 ROS2 fork
  （`TommyBrownson/FAST-Calib_Ros2`、`MaroSalah02/FAST-Calib-ROS2`），要么用
  `pip install rosbags` + `rosbags-convert` 转成 ROS1 bag。

## 5. 建图

```bash
ros2 launch handheld_mapping mapping_livo2.py \
    mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json
```

RViz 里固定帧是 `map`，常看：`/pgo/map`（回环后）、`/path`（原始轨迹）、
`/pgo/loops`（回环边）、`/cloud_registered`（当前帧）。`/handheld/map_raw`
是没做回环的累积图，默认关掉；回环前后差别大不大，一看就知道是前端漂还是
后端没闭合。

**存图**（正常走 Ctrl-C 也会存，见下），也可以随时手动存：

```bash
ros2 service call /pgo/save_map std_srvs/srv/Trigger
# 默认写 $HOME/pgo_map_<时间戳>.pcd；也可以指定路径
ros2 launch handheld_mapping mapping_livo2.py pcd_path:=$HOME/maps/room1.pcd
```

一次存图给出**三份**交付物，都以 PCD 的名字为前缀：

| 文件 | 内容 |
|---|---|
| `<name>.pcd` | 优化后的点云（`map` 系，按 `map_voxel` 采样，当前 0.005 m） |
| `<name>_traj.tum` | 优化后的关键帧轨迹，TUM 格式 `stamp tx ty tz qx qy qz qw` |
| `<name>_loops.csv` | 通过检验的回环边 `from,to,fitness,inlier_ratio` |

两个配置项控制它（`config/pgo_handheld.yaml` / `pgo_limo.yaml`，默认都开）：
`save_extra_outputs: true` 写上面那两份文本；`save_on_shutdown: true` 让节点在
**Ctrl-C 时自动存一遍**（`pcd_path` 空就落到 `$HOME/pgo_map_<时间戳>.pcd`）。
手持时手一滑按了 Ctrl-C 不至于白走一趟——不过还是建议走完后主动存一次，
因为关机的存档用的是关机那一刻的图。

走位建议（对建图精度影响比参数大得多）：

- 起步后**站稳 2 秒**：FAST-LIVO2 用前 50 帧 IMU 定重力和零偏，晃动会让整个
  世界系歪掉。
- 走慢、转弯更慢（手持时角速度别超过 ~0.5 rad/s），LiDAR 是 10 Hz。
- 想闭合就**绕回起点**，回环用半径 3 m / 180° 的搜索，路线间距别超过几米。
- 纯白墙、玻璃、强逆光区域 VIO 帮不上忙，靠 LIO，几何结构要丰富。
- 走完一圈回来别急着停，多站一会儿让回环跑完再存图。

只跑 LIO（不接相机、或相机是坏的）：

```bash
ros2 launch handheld_mapping mapping_livo2.py use_camera:=false img_en:=0 \
    mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json
```

`img_en:=1` 而相机没数据时 FAST-LIVO2 **不会报错、也不会出轨迹**（在等属于
那一帧的图），所以 launch 里做了拦截：`use_camera:=false` 时 `img_en` 必须给 0。

### 5.1 终端平时不出声，位姿一跳就报一行

后端默认**安静**（`verbose: false`）：建图过程中不打印关键帧、不打印出图/回环/GNSS
的任何一行。还在打的只有三类：**警告和错误**、**位姿跳变报告**、**存图确认**。

FAST-LIVO2 丢掉 VIO 锁之后**不会停、也不会说**：它退回只积分 IMU，位姿以几十到
几百 m/s 飞出去（2026-10-26 那次跑到了 −7 km），而且不会自恢复。所以后端留了一个
**只报告不拦截**的位姿跳变检测（`config/pgo_handheld.yaml` 里的 `sanity_*`）：

| 参数 | 默认 | 含义 |
|---|---|---|
| `verbose` | `false` | 实时日志总开关 |
| `sanity_enabled` | `true` | 位姿跳变检测开关（只打印） |
| `sanity_refuse_scans` | `false` | `true` = 跳变的帧不再建图（旧行为） |
| `sanity_max_speed` | `3.0` m/s | 相对上一个被接受位姿的速度上限 |
| `sanity_max_jump` | `0.5` m | 相对上一个被接受位姿的单次位移上限 |

跳变时打一条（节流：第一条立即，之后每 2 s 一条）：

```
Pose jumped 120.61 m in 0.698 s (= 172.9 m/s, limits 0.50 m / 3.0 m/s);
7 in a row, 7 total. The scan is being mapped anyway -- if this is a real
divergence, stop and re-run the survey.
```

**默认这帧照样建图。** 实机上这套阈值误报太多（快转身、丢帧都会触发），拦掉那几帧
对图反而更糟；但"图悄悄地被飘掉的帧填满"更糟，所以保留报告、不保留拦截。
真要拦截就把 `sanity_refuse_scans` 改成 `true`——此时跳变的帧整帧丢掉（不进关键帧、
不做融合、不搜回环），并在连续 `sanity_reacquire_scans` 帧正常增量、且离上一个好位姿
不超过 `sanity_reacquire_jump` 后重新接管。

**这道检测治不了前端**：它不能让 LIVO2 恢复、不能重启、也不能告诉你该往回走到哪。
看到它连续打，正确动作是 Ctrl-C 存图、重走一遍。

排查时想临时看全量日志，不用改配置重编，直接对运行中的节点开：

```bash
ros2 param set /pgo_backend verbose true
```

### 5.2 前端崩了怎么留证据（默认在 gdb 下跑）

FAST-LIVO2 在实机上**会 segfault**（2026-10-26 20:29:39：`segfault at
fffffffffffffff4`，崩在 IMU 回调那一片，它自己的日志最后几行完全正常，没有任何报错；
apport 也没留 core，`/var/crash` 里什么都没有）。前端一死，`/aft_mapped_to_init`
停发，后面整条链——bridge、`/cloud_registered`、PGO 的图、轨迹、rviz——全部静止。

所以 launch 现在**默认把 LIVO2 套在 gdb 里跑**（`launch/_common.py` 的
`livo2_front_end(..., gdb=True)`）。正常运行时毫无区别，Ctrl-C 照常终止；一旦
segfault，gdb 会把全线程栈和寄存器写进同一个日志：

```bash
grep -n "Program received signal" -A 120 /tmp/fastlivo_mapping.log
```

拿到栈就能定位到具体函数（release 版没有行号，但函数名和调用链是准的；要行号得
`-DCMAKE_BUILD_TYPE=RelWithDebInfo` 重编 fast_livo）。

不想用 gdb 时：`LIVO2_NO_GDB=1 ros2 launch handheld_mapping mapping_livo2.py ...`。

rviz 被压到 `warn` 级（`launch/_common.py` 的 `rviz()`），所以终端不再刷
`Message Filter dropping message: frame 'camera_init' ... earlier than all the data in
the transform cache`——那是 rviz 自己画不出 latched 老消息（戳比 10 s TF 缓存还早），
跟建图无关。想彻底安静就把那一行的 `'warn'` 改成 `'error'`。

### 5.3 只有雷达时：fast_limo 冒烟跑（不需要标定板）

fast_limo 是**纯 LiDAR + IMU**，不用相机，所以它不需要相机内参、不需要
LiDAR-相机外参、也不需要那块大标定板。在标定板做好之前，这是验证
「驱动 / 时间戳 / IMU / 外参 / Zenoh / TF」整条链的办法，也是以后定位备胎。

```bash
ros2 launch handheld_mapping mapping_limo.py
# 等价于带上 mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json
# （launch 会优先用这个文件，没有才退回 share 里那个占位配置）
```

没有 PGO 后端，也没有相机：`/fast_limo/final_raw`（全密度、odom 系）由
`map_accumulator` 累加成 `/handheld/map_raw`，就是「像仿真里那样直接累当前帧」。

**坐标系约定**（和 vectormachines 一致）：

```
map --静态恒等--> odom --fast_limo--> base_link --静态恒等--> livox_frame
```

- `base_link` **就在雷达中心**，朝向与雷达一致（`fast_limo_handheld.yaml` 的
  `extrinsics.lidar.t` 因此是 0，IMU 反过来挪到出厂值 +2.6 cm 处）。所以 RViz 里
  那组坐标轴**就是传感器本体**，不是离它 2.6 cm 的 IMU 体。
- `map→odom` 是静态恒等，所以**起始时 base_link 的原点正好在 map 原点**，
  之后跟着人走。TF 显示里 map / odom / base_link 各有一个开关；
  `livox_frame` 默认关（它和 base_link 完全重合，只在要显示原始
  `/livox/lidar_limo` 时才需要它——launch 里给了静态恒等变换，否则 RViz
  会因为找不到坐标系而拒画）。
- **起始姿态不是单位四元数**：`gravity_align` 让世界系的 z 轴对齐重力，
  而不是对齐开机那一刻的雷达姿态。所以把架子斜着放（比如俯仰 26°）时，
  起点位置仍在原点，但三根轴会**歪着**——那 26° 就是架子的真实倾角。

看的东西：`/fast_limo/state`（200 Hz 的 odom）、`/fast_limo/final_raw`（当前帧）、
`/handheld/map_raw`（累积图，1 Hz）、`/handheld/trajectory_raw`（轨迹）。
RViz 用 `rviz/mapping_limo.rviz`。

**手持时滤掉 1 m 以内的点**：`min_range`（launch 参数，默认 `1.0`）让适配节点把
离传感器不到 1 m 的点从**两路输出**里剔掉。手持时那 1 m 里全是**你自己**——
手柄、支架、胳膊。它们不是可以被平均掉的噪声：它们**跟着传感器一起动**，在匹配器眼里
就是静止结构，所以会顺着轨迹在建好的图里拖出一条鬼影。滤波在适配节点里做，早于
所有前端，所以 LIVO2 的 LIO/光度项、fast_limo、`/cloud_registered`、
`/fast_limo/final_raw`、`/pgo/map` 全都是干净的。
判据是**到雷达中心的球面距离**（`base_link` 就在雷达中心，所以就是「离雷达 1 m」）。

- **两路都滤**：`/livox/lidar_livo`（LIVO2）和 `/livox/lidar_limo`（fast_limo）。
  `mapping_livo2.py` 和 `mapping_limo.py` 的 `min_range` 默认都是 `1.0`。
- 关掉：`ros2 launch handheld_mapping mapping_livo2.py min_range:=0.0`
  （或 `mapping_limo.py` 同样写法）
- 日志每 `stats_period` 秒报一次：
  `2000 points, 1000 kept after min_range 1.00 m (1000 dropped)`
- 边界是 `<`：正好 1.000 m 的点**保留**。
- LIVO2 自己还有一道 `preprocess.blind`（`livo2_mid360.yaml` 里 0.1 = 10 cm，
  代码里平方后比较）——那是雷达盲区门，只挡贴脸的死点，别指望用它代替 `min_range`。

**起步 15 秒不建图**：`config/map_accumulator_limo.yaml` 的 `warmup_seconds: 15.0`
（`map_accumulator_node` 的参数，默认 0.0 = 关，仿真那条链不受影响）。窗口从**第一帧
点云到达**开始算，不占用前面 5 s 的 IMU 标定时间；窗口内的帧直接丢掉，不进体素表——
体素表只会新增不会删除，脏帧进去就是永久的鬼影。日志里能看到
`Warm-up gate: ...` 和 `Warm-up finished after 15.0 s of scans`。
计时用的是**帧自带的时间戳**而不是墙钟（bag 回放时也对），代价是：实测 fast_limo
起步头几帧的戳比到达时刻超前约 4–5 s，所以按墙钟看实际会跳掉约 20 s——宁可多跳。
轨迹（`/handheld/trajectory_raw`）不受这个门影响，从头就录。

**两个实机才暴露的坑，都已经处理掉，但要知道为什么：**

- **时间戳是两套约定。** fast_limo 的 LIVOX 分支把逐点 `timestamp` 当作
  「相对扫描起点的**秒数**」，直接加在 cloud 的 header 上再拿去做 IMU 状态插值
  （`Localizer.cpp` 的 `sweep_ref_time + pt.timestamp`）；而 LIVO2 的
  `mid360_handler` 要的是同一字段里的**绝对纳秒**。喂错单位不会报错，只会让每个点
  去查一个 1.79e18 秒之后的姿态，一秒内姿态飞到几百米开外。所以适配节点现在同时发
  两个话题：`/livox/lidar_livo`（绝对纳秒，给 LIVO2）和 `/livox/lidar_limo`
  （相对秒，给 fast_limo），几何完全相同。
- **累积图没有强度。** `map_accumulator` 内部点型是 `pcl::PointXYZRGB`，fast_limo
  的点只有 intensity，所以每帧都会打一行 `Failed to find match for field 'rgb'`
  —— 是噪音不是错误；`/handheld/map_raw` 出来是 rgb 全 0 的无色点云，
  `rviz/mapping_limo.rviz` 里那张图因此用 FlatColor 而不是按强度着色。
  当前帧 `/fast_limo/final_raw` 仍然有 intensity，可以按强度看。

实机第一次跑通的基线（2026-10-25，仪器静止在地面）：5 s 标定门过后 `Livox @
10.00 Hz / IMU @ 204.5 Hz`，静止 30 s 位置漂 **1.5 mm**，`/handheld/map_raw`
约 5.7 万点、96% 的点在 5 m 内（剩下的是窗外/玻璃的长距离杂点）。

**回环后端已经接在这条链上了**（`use_pgo:=true`，默认开；`use_pgo:=false`
回到只有原始累积的冒烟跑）。同一个节点、同一套因子，配置另存为
`config/pgo_limo.yaml`，和 LIVO2 用的 `pgo_handheld.yaml` 只差三处：

| 参数 | 值 | 为什么 |
|---|---|---|
| `frontend_cloud_topic` | `/fast_limo/final_raw` | 不能用 `/fast_limo/pointcloud`：它已被匹配器压到 0.1 m，比关键帧体素 0.05 还粗 |
| `odom_topic` | `/fast_limo/state` | 这条链里没有任何东西叫 `/odometry/lio`，不 remap |
| `deskew` | `false` | fast_limo 发出来之前已经去畸变；节点日志自己会写 `(front end, already undistorted, deskew=off)` |

输出 `/pgo/map`、`/pgo/trajectory`、`/pgo/loops`，并且**独占 `map→odom`**——
所以 `use_pgo:=true` 时 launch **不再发**那条静态恒等变换（两个发布者 = 抖动）。
第一个关键帧是「永远接受」的，所以前端一出点云 `map→odom` 就有了（先是恒等），
在这之前 TF 树上没有 `map→odom`，rviz 会抱怨几秒，正常。

两个使用注意：

- `/pgo/map` 的 QoS 是 volatile、只在有**新关键帧**时重建并发布。站着不动时后端
  只收 `kf 0` 一帧，之后不再发——「后开 rviz 看不到 PGO 地图」不是坏了，动一下就有。
- `/pgo/map` 和 `/handheld/map_raw` 都**没有颜色**（这条链看不到图像），rviz 里
  两张图都用 FlatColor，一个绿色（PGO）一个浅蓝（原始累积），叠在一起就能看出
  回环把哪一段扳回来了。

后端本身很便宜：实测 `pgo_backend` 约 **3% CPU**（fast_limo 约 90%、累积器 15%），
而且**这条开销不随建图时长增长**——每个关键帧只在被接受时融合一次（体素哈希里累加
位置/强度/颜色），发布时只是把格子物化（取均值），不再每次发布都把所有关键帧重算一遍。
唯一 O(总点数) 的操作是**位姿图优化之后的重建**（优化会同时移动很多关键帧，已经融合
的格子没法反融合），它一次回环只发生一次，而不是每两秒一次。仿真链的
`pgo_handheld.yaml` 走的是同一套代码。
静止时只跑出 `kf 0`，**回环闭合需要走动+回到原地**才能验证，这一项还没测过。

其它可调项：关键帧 0.20 m / 15°（`pgo_limo.yaml`）、回环半径 3 m、
ICP 内点率 0.60 / fitness 0.10。前端参数（`config/livo2_mid360.yaml` 的
`vio` / `lio` 段）与仿真一致，定位正常就别动，建图问题优先在后端/走位上找。

## 6. 为什么需要 `livox_livo2_adapter`

两件不相干的事，都是因为「驱动发的东西，FAST-LIVO2 恰好吃不了」。

**（1）消息类型对不上（这个坑是静默的，先看这条）**

`config/livo2_mid360.yaml` 用的是 `lidar_type: 8`（MID360 预处理器），它在
`LIVMapper.cpp` 里注册的是 **`sensor_msgs/PointCloud2`** 订阅，而驱动发的是
`livox_ros_driver2/CustomMsg`。类型不同的两个端点不会「报错不匹配」，而是**根本
绑不上**：LIVO2 一个点云都收不到，不打印任何错误、不出轨迹，只有 `/image_raw`
还在被消费。表现就是「图像明明在进，却什么都没有」。查的方法：

```bash
ros2 topic info /livox/lidar_livo -v      # 看 PUB 和 SUB 的 type 是否一致
```

`mid360_handler` 要的点云结构是 `x y z intensity tag line timestamp`，其中
`timestamp` 是**该点的绝对纳秒时间戳**（它算 `pt.timestamp * 1e-9 - header_time`
得扫描内偏移）。所以适配节点把 CustomMsg 逐点换算成
`timestamp = timebase + offset_time`，其余字段原样搬过去。

> **不要**改用驱动自己的 PointCloud2（`xfer_format: 0`）：那个 `timestamp` 字段写的是
> **相对扫描起点的秒数**（`(offset_time - base_time)/1e9`），LIVO2 会把它当成
> 1.7e9 秒以前的时间点，去畸变直接算飞。

**（2）时间基要对得上相机**

`livox_ros_driver2` 1.2.4 的 `GetEthPacketTimestamp` 在**没有 PTP/GPS 同步**时
（手持就是这种情况）返回的是 `high_resolution_clock::now()`，也就是**收包时刻的
主机时间**——本来就和相机同一个钟，不需要平移。只有走 PTP/GPS 时它才返回设备时钟。
实测印证：`/livox/lidar` 的 header（扫描起点）比收包时刻早约 145 ms，那正好是
「扫描 100 ms + 发布延迟」，不是时钟差。

即便如此，适配节点仍然测一次偏移，因为出问题的方向是静默的（设备时钟会差 1.7e9 秒，
图像永远配不上扫描，LIVO2 一直等、无输出、无报错）：

- 偏移 = `median(host_now - (timebase + 最后一个点的 offset_time))`，用扫描**结束**
  时刻消掉约 100 ms 的扫描时长偏置；先丢 `warmup_frames`（默认 5）帧，因为驱动刚连上
  会补发积压（实测那几帧的到达时间抖动 2.4 s，稳定后只有 20 ms）。
- 绝对值小于 `host_time_threshold`（默认 10 s）= 本来就在主机时钟 → 偏移设 0，原样透传；
  大于它就说明是设备时钟 → 加到扫描 header、逐点时间戳和 IMU 上。
- 只动时间，不动点本身。残余漂移由 LIVO2 的 `td_estimate_enable` 吸收。

相机这边：图像 stamp 用的是 **V4L2 buffer 的采集时间戳**（`buf.timestamp` + epoch 平移），
所以它是诚实的采集时刻，LIVO2 按戳关联，**延迟本身不影响精度，只影响实时性**。

**（2b）相机节点为什么不用 usb_cam**

`usb_cam` 0.8.1 在本机把图像延迟到 **0.1～1.0 s**：11 次独立启动分别得到 98 / 229 / 303 /
406 / 422 / 456 / 636 / 757 / 896 / 1008 / 1011 ms。这个数**一次启动定死**（σ 2～6 ms，
70 s 不漂），而且跟订阅侧无关：RELIABLE / BEST_EFFORT、depth 1 / 10 全一样；两个不同时刻
接入的订阅者看到完全相同的值；运行中把它的计时器从 33 ms 改成 16 ms（`framerate:=60`）
也纹丝不动。同一台相机、同样 4 个 mmap buffer，换成最小 V4L2 读取程序只有 **33～48 ms**，
`v4l2-ctl` 也是。所以问题在 usb_cam 自己的采集循环：它**每个 timer tick 只 DQBUF 一次**，
一旦积压就永远排不空，EAGAIN 时还会拿上一帧+旧戳重发（`io_method: read` 在 0.8.1 里直
接 core dump，别用）。

因此建图链路改用本包自己的 **`v4l2_capture`**（`src/v4l2_capture.cpp`）：`poll()` 等到有帧
→ 把队列**抽干**、只留最新一帧、其余立刻归还 → 立即发布，没有计时器也没有排队。
实测 **36 ms（33～41 ms，σ 2，70 s 不漂）**，Zenoh / Fast DDS 一样，30 fps 稳定。
同一个话题、同一种编码（`yuv422_yuy2`）、同一种诚实戳，下游一行不用改。

- 每 10 s 一行健康指标：`29.8 fps, frame age at publish 32-37 ms, 0 stale frame(s) discarded`。
  `frame age` 涨起来才是问题；`stale discarded` 是它主动丢掉的旧帧（说明确实在抽干）。
- **不写任何 V4L2 控件**——曝光等交给 `set_camera_controls.sh`，不会互相覆盖。
- **不发 `/camera_info`**，所以 `calibrate_camera.py` / `calibrate_lidar_camera.py` 仍然用
  `usb_cam`（标定链路不变）。`sensors.py` 和 `mapping_livo2.py` 用的是 `v4l2_capture`。

**（3）这个节点必须是 C++**

它原本是 Python（`rclpy`）。问题不在转换本身（实测 6.7 ms/帧，7% 单核），而在
**反序列化**：一帧 CustomMsg 要在 Python 里造两万个点对象，光这一项就吃掉大半个核，
于是整个节点的处理速度**略低于**输入速率。订阅是 RELIABLE / depth 200，欠的那一点
永远不会被丢掉，只会排队——实测 IMU 到 LIVO2 的延迟从 +0.05 s 一路涨到 +4.8 s
（85 s 内），随后 `sync_packages` 看到「图像比最新雷达新」就永远返回 false，
LIO/VIO 全部停摆（图上看就是「什么都没出」），积压回放阶段还会 SIGSEGV。

C++ 版把这块压到 1% 量级。所以现在 `src/livox_livo2_adapter.cpp` 是**唯一编译**的
东西，`scripts/` 里只剩工具脚本。节点每 `stats_period`（默认 10 s）打一行
`sweeps/s / points / 落后主机时钟多少 ms`，**这一行就是判断它跟不跟得上的指标**：
`sweeps/s` 掉到 10 以下、或延迟随时间长，说明哪里堵了。

> 别用 Python 版把它换回来，也别靠调大 QoS depth「扛一下」——那是把延迟变成无界队列。

## 7. 录包与回放

```bash
ros2 launch handheld_mapping record.py bag_out:=$HOME/bags/room1
# 录的是适配之后的话题 + /tf + /tf_static，不录驱动的原始话题
```

回放（先关掉真实传感器，用 bag 的 /clock）：

```bash
ros2 launch handheld_mapping mapping_livo2.py \
    use_lidar:=false use_camera:=false use_sim_time:=true
ros2 bag play ~/bags/room1_0 --clock
```

## 8. 常见坑

| 现象 | 原因 |
|---|---|
| FAST-LIVO2 不输出、也不报错，但图像在进 | 十有八九是**消息类型绑不上**：`lid_topic` 上 PUB 的 type 和 SUB 的 type 不同（`ros2 topic info -v`）。另外见下一条 |
| FAST-LIVO2 不输出、也不报错（类型对得上） | `img_en:=1` 但没图像；或时间戳没对齐（看第 3 节） |
| 启动即 SIGFPE | `parameter_blackboard` 没起（vikit 从它读内参，读到 0） |
| 点云没上色 / 全是灰 | `image_en` 关了，或外参完全错 |
| 地图有双层墙 | 外参不准 / 计时偏差；先按第 4.2 节重标 |
| 建图一开始就歪 | 起步前晃动了，重来并站稳 2 秒 |
| RViz 只有 `map` 没东西 | 固定帧是 `map`，需要 PGO 在跑；`use_pgo:=false` 时用静态 `map->odom` |
| 驱动收不到点 | IP 配置 / 网卡网段 / UDP 56100-56500 被防火墙挡 |
| `/image_raw` 一直不出 | 相机节点叫 `v4l2_capture`（不是 usb_cam）；看它的日志有没有 `capturing 640x480 ...`，没有就是 `/dev/video0` 被别的东西占着 |
| 图像比现实明显慢半拍 | 用的还是 usb_cam。换成 `v4l2_capture`（见 6.2b），或 `ros2 topic delay /image_raw` 确认 |
| 标定时报 `/usb_cam` unknown control | 只有标定链路用 usb_cam；它的参数名（autofocus 等）在这台相机上不存在，用 `set_camera_controls.sh` 走 V4L2 |
