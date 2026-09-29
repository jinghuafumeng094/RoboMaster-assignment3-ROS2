# RoboMaster assignment3 ROS2

这份仓库提供一个 ROS 2 相机功能包，在 Ubuntu 22.04 / ROS 2 Humble 上，基于海康机器人 MVS SDK 实现相机接入、图像发布、参数动态设置和断线重连。

## 开始

1. 点击 GitHub 页面右上角的 **Fork**，将仓库复制到你的账号下。
2. 在你的 Fork 页面点击 **Code**，复制地址并克隆到本地：

   ```bash
   # 将下面的地址替换为你的 Fork 地址
   git clone <你的 Fork 地址>
   cd robomaster-camera-assignment
   ```

3. 阅读 [ROS 2 教程](docs/ROS2Tutorial.md) 和 [作业要求](docs/assignment.md)。
4. 按下面的步骤构建并启动工程。

## 仓库结构

```text
robomaster-camera-assignment/          # 同时作为 colcon 工作空间
├── AGENTS.md
├── README.md
├── docs/
│   ├── ROS2Tutorial.md
│   └── assignment.md
└── src/hikrobot_camera/              # ROS 2 功能包
    ├── package.xml                   # 包信息与依赖
    ├── CMakeLists.txt                # 构建与安装配置
    ├── include/hikrobot_camera/
    │   ├── camera_node.hpp          # 节点声明
    │   └── mvs_camera.hpp           # MVS SDK 封装声明
    ├── src/
    │   ├── main.cpp                 # 程序入口
    │   ├── camera_node.cpp          # 节点实现
    │   └── mvs_camera.cpp           # SDK 封装实现
    ├── launch/camera.launch.py      # 启动文件
    └── config/camera.yaml           # 参数配置
```

## 环境与依赖

需要 Ubuntu 22.04、ROS 2 Humble，以及海康机器人 MVS SDK。

### ROS 2 Humble

参考 [ROS 2 Humble 安装文档](https://docs.ros.org/en/humble/Installation.html)。

安装后确认以下工具可用：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
colcon --version
rosdep --version
```

### MVS SDK

1. 从 [海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0) 下载 Linux x86_64 版 MVS SDK。

2. 安装后确认以下路径存在：

   - 头文件：`/opt/MVS/include/MvCameraControl.h`
   - 库文件：`/opt/MVS/lib/64/libMvCameraControl.so`

3. 若 SDK 安装在非默认位置，编译时用环境变量指定：

   ```bash
   MVS_ROOT=/your/mvs/path colcon build --packages-select hikrobot_camera
   ```

4. ARM 平台可用 `MVS_LIB_SUBDIR` 指定库子目录名（默认 `64`）：

   ```bash
   MVS_LIB_SUBDIR=aarch64 colcon build --packages-select hikrobot_camera
   ```

### ROS 依赖

本仓库本身就是 colcon 工作空间，在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
# 仅当系统尚未初始化 rosdep 时执行一次：sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## 编译

```bash
cd robomaster-camera-assignment
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hikrobot_camera
```

编译时会自动在可执行文件中写入 rpath，运行时无需手动设置 `LD_LIBRARY_PATH`。

## 运行

另开终端，在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hikrobot_camera camera.launch.py
```

如果你使用 Zsh，将环境脚本的 `.bash` 换为 `.zsh`。

启动后，节点会：

1. 打开 index 0 的相机（或按配置的序列号 / IP）；
2. 把图像发布到 `/image_raw`；
3. 加载 `config/camera.yaml` 中的参数。

可以用自己的参数文件覆盖默认配置：

```bash
ros2 launch hikrobot_camera camera.launch.py params_file:=/absolute/path/to/camera.yaml
```

## 在 RViz2 中查看图像

```bash
rviz2
```

操作步骤：

1. `Fixed Frame` 设为 `camera_optical_frame`；
2. 点击 `Add`，选择 `Image`；
3. `Image Topic` 填 `/image_raw`。

图像会实时显示。

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `serial_number` | string | `""` | 按序列号选相机，留空则按 index |
| `ip_address` | string | `""` | 按 IP 选相机（仅 GigE），优先级高于 `serial_number` |
| `topic_name` | string | `image_raw` | 发布图像的话题名 |
| `frame_id` | string | `camera_optical_frame` | 图像消息的 frame_id |
| `pixel_format` | string | `bgr8` | `bgr8` / `bayer_rggb8` |
| `exposure_time` | double | `5000.0` | 曝光时间，微秒 |
| `gain` | double | `0.0` | 增益，dB |
| `frame_rate` | double | `30.0` | 目标帧率，Hz |

运行时动态修改参数：

```bash
ros2 param set /hikrobot_camera exposure_time 10000.0
ros2 param set /hikrobot_camera gain 5.0
ros2 param set /hikrobot_camera frame_rate 45.0
ros2 param set /hikrobot_camera pixel_format bayer_rggb8
```

每次修改后参数会立即下发到相机。若 SDK 设置失败或读回值不匹配，ROS 2 参数更新会被拒绝并回滚。

## 在这里解释你的项目

### 1. 项目功能

- **设备枚举与选择**：支持按 index、序列号、IP（GigE）三种方式打开相机。设备不存在、序列号冲突或设备被占用时返回明确错误。
- **图像采集与发布**：采集相机图像并发布 `sensor_msgs/msg/Image` 消息到可配置话题。
- **像素格式控制**：支持 `bgr8` /`bayer_rggb8` 两种格式，切换时直接修改相机内部 `PixelFormat` 节点。
- **参数动态设置**：通过 ROS 2 参数控制曝光时间、增益、帧率、像素格式。set 操作带读回验证，失败会回滚并报告原因。
- **断线重连**：独立重连线程，连续抓帧失败后自动尝试重连，重连成功后从 ROS 2 参数恢复最新配置。
- **资源释放**：退出时按顺序停线程、停取流、关设备、销毁句柄。

### 2. 如何编译

见「编译」章节。核心命令：

```bash
colcon build --symlink-install --packages-select hikrobot_camera
```

### 3. 运行方式

见「运行」章节。核心命令：

```bash
ros2 launch hikrobot_camera camera.launch.py
```

### 4. 代码结构

代码分为两层：

| 层 | 文件 | 职责 |
|---|---|---|
| **SDK 封装层** | `mvs_camera.hpp` / `mvs_camera.cpp` | 封装海康 MVS SDK 调用，管理相机句柄，所有 public 方法加 `std::mutex` 保护 |
| **ROS 2 节点层** | `camera_node.hpp` / `camera_node.cpp` | 处理参数声明、话题发布、参数回调、断线重连逻辑 |

**核心方法 `grabFrame`**：一次加锁完成 `MV_CC_GetImageBuffer` → 拷贝数据 → `MV_CC_FreeImageBuffer`，确保外部拿到的是独立数据，不引用 SDK 内部缓冲区。

### 5. 断线重连机制

- 主线程通过 `timerCallback` 每 10ms 抓一帧，连续失败 10 次触发重连；
- 独立重连线程每 200ms 轮询一次标志，需要时执行：`close` → 等 500ms → 重新打开 → 恢复参数 → `startGrabbing`；
- 主线程与重连线程通过 `std::atomic` 变量通信；
- 重连成功后从 `get_parameter()` 读最新参数值，恢复相机配置。

### 6. 几点说明

| 项 | 说明 |
|---|---|
| **帧率上限** | 本节点用 10ms 周期定时器抓帧，理论发布上限 100 Hz。同时 `frame_rate` 还受曝光时间限制（曝光 20ms → 上限 50Hz）。`ros2 topic hz /image_raw` 可查看实际频率，与设置值可能不一致，差异来自曝光时间、USB 带宽、等因素。 |
| **图像时间戳** | 使用 ROS 2 系统时间（`this->now()`），表示节点处理时刻，不是相机采集时刻。|
| **只支持 2 种像素格式** | 相机还支持 mono_8，YUV422，但这些格式没有对应的 ROS 2 标准 encoding，未实现。 |

## 完成与提交

代码已完成，源代码、Launch 和参数配置已推送到 Fork。

## 提交信息

- 格式：第三次作业-人工智能2601-黄鑫睿
- 发送至：2719850558@qq.com