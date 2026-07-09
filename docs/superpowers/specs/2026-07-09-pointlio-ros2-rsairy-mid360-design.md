# Point-LIO ROS2 迁移 + RSAIRY/MID360 适配设计文档

**日期**: 2026-07-09
**仓库**: TsingYun-RoboMaster/Point-LIO (fork of hku-mars/Point-LIO)
**分支**: `ros2-mid360-rsairy`

## 1. 目标

将 Point-LIO 从 ROS1 Noetic 迁移到 ROS2 Humble，删除旧 LiDAR 接口（AVIA/Livox、VELO16、OUST64、HESAIxt32），新增 RSAIRY 和 MID360 两种 LiDAR 支持。

## 2. 两种架构方案

### 2.1 方案 A：帧级处理（原样保留 Point-LIO 架构）

保持 Point-LIO 现有帧级架构不变。收齐完整一帧点云 → handler 转换 → 特征提取（give_feature/plane_judge）→ IEKF 批量更新。

#### 数据流

```
RSAIRY: rslidar_sdk → PointCloud2(x/y/z/intensity/ring/timestamp) → rsairy_handler → PointCloudXYZI
MID360: livox_ros_driver2 → CustomMsg                               → mid360_handler  → PointCloudXYZI
```

- 每个 handler 子类化已有的 `process()` 模式
- 接收帧级点云消息，转换为内部 `PointType`（曲率字段存逐点时间戳 ms）
- 帧完成后触发 `process_cut_frame_pcl2` → `give_feature` → EKF

#### 改动范围

| 文件 | 改动 |
|------|------|
| `CMakeLists.txt` | catkin → ament_cmake，依赖替换 |
| `package.xml` | format 2 → format 3 |
| `config/rsairy.yaml` | 新增 RSAIRY 配置 |
| `config/mid360.yaml` | 新增 MID360 配置（替代 iavia.yaml） |
| `launch/mapping_rsairy.launch.py` | 新增 ROS2 Python launch |
| `launch/mapping_mid360.launch.py` | 新增 ROS2 Python launch |
| `src/preprocess.h` | 删除 AVIA/Velodyne/Ouster/Heasi 枚举和 handler 声明；新增 `rsairy_pcl_point` 结构体（含 timestamp + ring）；新增 RSAIRY handler 声明 |
| `src/preprocess.cpp` | 删除旧 handler 实现；新增 `rsairy_handler()`；新增/保留 `mid360_handler()`；保留 `give_feature/plane_judge/edge_jump_judge` |
| `src/laserMapping.cpp` | ROS API 替换（roscpp→rclcpp，tf→tf2），主循环不变 |
| `src/Estimator.cpp/h` | 头文件路径替换 |
| `src/IMU_Processing.cpp` | 头文件路径替换 |
| `src/parameters.cpp/h` | `ros::NodeHandle` → `rclcpp::Node`，新增 RSAIRY/MID360 参数 |
| `rclcpp` 版本替换 | 全局 `ros::` → `rclcpp::`，`sensor_msgs` PCL 迭代器路径变化 |

### 2.2 方案 B：Packet 级处理（参照 XSmall_point_lio）

采用逐 packet 流式处理，不再等待完整帧。每个 packet 到达后立即解码 → 滤波 → 逐点 EKF 更新。特征提取改为滑动窗口增量方式。

#### 数据流

```
RSAIRY: rslidar_msg::RslidarPacket → rs_driver 解码 → PointXYZIRT → 通用 Point → preprocess → EKF
MID360: livox_ros_driver2::CustomMsg → 直接提取    → 通用 Point → preprocess → EKF
```

#### 改动范围

| 文件 | 改动 |
|------|------|
| 构建系统 | 同方案 A，额外新增 `rs_driver` 子目录编译、`rslidar_msg` 依赖 |
| `src/lidar_adapter/robosense_packet.h` | 新增，直接采用 XSmall 的实现 |
| `src/lidar_adapter/mid360_adapter.h` | 新增，CustomMsg → Point[] |
| `src/preprocess.cpp/h` | 整体重构为流式管线（参照 XSmall）：timestamp/range/reflectivity filter → ring stratified sampling → voxel downsample → sort → enqueue |
| `src/small_point_lio/lio.cpp/h` | 新增，interleaved IMU+point 主循环（参照 XSmall） |
| `src/laserMapping.cpp` | 不再需要，被 lio.cpp 替代 |
| 特征提取 | 需改造为滑动窗口增量模式（新实现） |

### 2.3 方案对比

| 维度 | 方案 A（帧级） | 方案 B（Packet 级） |
|------|---------------|---------------------|
| 与原版差异 | 最小，架构不变 | 大，架构重构 |
| 特征提取 | 原样保留，成熟稳定 | 需重新设计滑动窗口版本 |
| 延迟 | 帧级延迟（~100ms） | Packet 级延迟（~1ms） |
| 运动畸变 | 需处理 | 天然消除 |
| 代码复杂度 | 低（handler 照搬模式） | 中（需设计滑动窗口 + adapter） |
| 后续扩展新 LiDAR | 加一个 handler | 加一个 adapter |
| 审阅工作量 | 小，改动集中 | 大，新增大量新文件 |

## 3. 选定的实现计划（方案 A）

用户选择方案 A：帧级处理，保持原架构。

### 3.1 Step 1 — 分支与构建系统

**新分支**: `ros2-mid360-rsairy` 从 `point-lio-with-grid-map` 切出

**CMakeLists.txt 改写为 ament_cmake**：

```cmake
cmake_minimum_required(VERSION 3.10)
project(point_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

ament_auto_add_executable(pointlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
)
target_include_directories(pointlio_mapping PRIVATE include)
target_link_libraries(pointlio_mapping Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(pointlio_mapping
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros pcl_conversions livox_ros_driver2
)
ament_auto_package(INSTALL_TO_SHARE config launch)
```

**package.xml 改为 format 3**：`<buildtool_depend>ament_cmake</buildtool_depend>`，所有 `<build_depend>` + `<run_depend>` 合并为 `<depend>`。

### 3.2 Step 2 — 删除旧 LiDAR handler

**preprocess.h**:
- 删除 `LID_TYPE` 枚举 (AVIA/VELO16/OUST64/HESAIxt32)
- 删除 `TIME_UNIT` 枚举
- 删除 PCL 自定义点结构体 `velodyne_ros::Point`、`ouster_ros::Point`、`hesai_ros::Point`
- 删除 `process_cut_frame_livox`、`process_cut_frame_pcl2` 声明
- 删除 `avia_handler`、`oust64_handler`、`velodyne_handler`、`hesai_handler` 声明
- 删除 `pub_func` 声明
- 删除 `pl_buff`、`typess`、`given_offset_time`、`time_unit_scale`、`SCAN_RATE` 等成员
- **保留**: `give_feature`、`plane_judge`、`edge_jump_judge`、`small_plane`、`PointType`、`PointCloudXYZI`、`N_SCANS`、`blind`、`det_range`、`point_filter_num`、`orgtype`

**preprocess.cpp**:
- 删除所有旧 handler 实现
- 删除 `process_cut_frame_livox`、`process_cut_frame_pcl2`
- 删除 `pub_func`
- 删除 `Preprocess` 构造函数中的旧参数初始化
- **保留**: `give_feature`、`plane_judge`、`edge_jump_judge`、`small_plane` 函数体不变

### 3.3 Step 3 — 新增 RSAIRY handler

**rs_driver 点云格式**（`PointXYZIRT`，PCL 注册）:

```cpp
// rs_driver 解码后由 rslidar_sdk 发布为 PointCloud2，字段为:
// x(FLOAT32), y(FLOAT32), z(FLOAT32), intensity(FLOAT32),
// ring(UINT16), timestamp(FLOAT64)
```

**新增 `rsairy_pcl_point`**（preprocess.h 或独立头文件）:

```cpp
namespace rsairy_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;  // 秒级绝对时间戳
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rsairy_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)
```

**新增 `rsairy_handler()`**（preprocess.cpp）:

参照 `velodyne_handler` 的模式，核心逻辑：
- `pcl::fromROSMsg` 反序列化 PointCloud2
- 遍历点云，逐点转换为内部 `PointType`（curvature = timestamp * 1000，即转为 ms）
- 逐点做 range filter + decimate
- 填入 `pl_surf`，按 curvature 排序
- 可选：如果 `given_offset_time=false`，用恒定角速度模型估算 `curvature`（RSAIRY 是机械式，角速度来自 rs_driver 的 rpm 参数，约 10Hz）

### 3.4 Step 4 — 新增 MID360 handler

MID360 通过 `livox_ros_driver2` 发布 `CustomMsg`，结构：

```
CustomMsg:
  point_num
  points[]:
    x, y, z          (float32)
    reflectivity     (uint8)
    offset_time      (uint32, ns)
    line             (uint8)
    tag              (uint8)
```

handler 参照原 `process_cut_frame_livox`：
- 提取 `x/y/z/reflectivity/offset_time/line`
- `tag & 0x30` 过滤噪声
- 转换为 `PointType`（curvature = offset_time / 1000000 → ms）
- 使用 `pl_buff` 缓存按 line 分组
- 调用 `give_feature` 特征提取

### 3.5 Step 5 — 预处理管线整合

`Preprocess` 类保留两个 `process()` 重载：
- `process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)` → 调用 `rsairy_handler()`
- `process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg)` → 调用 `mid360_handler()`

保留 `process_cut_frame_pcl2` 的帧切割逻辑（给 RSAIRY），保留 `process_cut_frame_livox` 的帧切割逻辑（给 MID360）。

特征提取管线 `give_feature → plane_judge → edge_jump_judge` 完全保持不变。

### 3.6 Step 6 — 状态估计核心 ROS API 替换

**laserMapping.cpp**：
- `ros::Subscriber` → `rclcpp::Subscription`
- `ros::Publisher` → `rclcpp::Publisher`
- `ros::Time::now()` → `node->get_clock()->now()`
- `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`
- `ros::NodeHandle` → `rclcpp::Node`
- 回调签名改为 `SharedPtr`
- 主循环用 `rclcpp::spin()` 或独立线程 + `rclcpp::Rate`

**Estimator.cpp/h**：头文件 include 路径调整（`pcl_conversions` 在 ROS2 中独立于 ros）

**IMU_Processing.cpp**：同上

**parameters.cpp/h**：`ros::NodeHandle::param<T>()` → `node->declare_parameter<T>()` + `node->get_parameter<T>()`

### 3.7 Step 7 — 节点入口 + Launch + 配置

**新增 `src/point_lio_node.cpp`**:
- 继承 `rclcpp::Node`
- 根据参数选择 LiDAR 类型（`lidar_type: "rsairy"` 或 `"mid360"`）
- 订阅对应 topic，组合 Preprocess + Estimator

**新增 `config/rsairy.yaml`**:
```yaml
point_lio_node:
  ros__parameters:
    lidar_type: "rsairy"
    lid_topic: "/rslidar_points"
    imu_topic: "/rslidar_imu_data"
    lidar_frame: "rslidar"
    N_SCANS: 96
    SCAN_RATE: 10
    timestamp_unit: "SEC"      # RSAIRY timestamp in seconds
    blind: 0.3
    det_range: 200.0
    point_filter_num: 1
    extrinsic_T: [0.0, 0.0, 0.0]
    extrinsic_R: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
    satu_acc: 30.0
    satu_gyro: 35.0
    acc_norm: 1.0             # IMU outputs in [g]
    # ... 更多参数
```

**新增 `config/mid360.yaml`**: 类似结构但针对 Livox MID360

**新增 `launch/mapping_rsairy.launch.py`** 和 **`launch/mapping_mid360.launch.py`**: Python ROS2 launch

### 3.8 保留/删除清单

| 文件 | 命运 |
|------|------|
| `config/avia.yaml` | 删除 |
| `config/velody16.yaml` | 删除 |
| `config/ouster64.yaml` | 删除 |
| `config/horizon.yaml` | 删除 |
| `launch/mapping_avia.launch` | 删除 |
| `launch/mapping_horizon.launch` | 删除 |
| `launch/mapping_ouster64.launch` | 删除 |
| `launch/mapping_velody16.launch` | 删除 |
| `launch/gdb_debug_example.launch` | 删除 |
| `include/matplotlibcpp.h` | 保留（调试用） |
| `include/IKFoM/` | 保留（核心数学库，不变） |
| `include/ivox/` | 保留（核心地图库，不变） |
| `include/so3_math.h` | 保留 |
| `include/common_lib.h` | 保留（调整 include 路径） |
| `src/li_initialization.cpp/h` | 保留 |
| `src/Estimator.cpp/h` | 保留（调 include 路径） |
| `src/IMU_Processing.cpp/h` | 保留 |

## 4. Commit 计划

| Step | Commit | 说明 |
|------|--------|------|
| 1 | `build: migrate to ament_cmake for ROS2 Humble` | CMakeLists.txt + package.xml |
| 2 | `refactor: remove legacy LiDAR handlers` | 删除旧 handler，保留特征提取核心 |
| 3 | `feat: add RSAIRY point cloud handler` | 新增 rsairy_pcl_point + rsairy_handler |
| 4 | `feat: add MID360 CustomMsg handler` | 新增 mid360_handler (ROS2 CustomMsg) |
| 5 | `refactor: migrate preprocessing to ROS2` | Preprocess 类 ROS2 API + 整合新 handler |
| 6 | `refactor: migrate state estimation core to ROS2` | laserMapping/Estimator/IMU_Processing/parameters ROS API |
| 7 | `feat: add node entry, launch files, and configs` | point_lio_node + rsairy.yaml + mid360.yaml + launch |

每个 commit 独立可编译（或有条件地可编译），改动尽量小。
