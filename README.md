# moma_nav

基于 ROS 1 move_base 框架的导航仓库，支持 GlobalPlanner、Hybrid A* 全局规划和 TEB、DWA 局部规划，提供代价地图配置与二维位姿投影节点，并集成可选的 EMA 速度平滑器。仓库内的导航功能包名称为 `moma_nav`，默认使用 GlobalPlanner 与 TEB。

## 仓库布局

```text
moma_nav/
├── README.md
├── moma_nav/                 # 调用 move_base 框架
├── teb_local_planner/        # TEB 局部规划器子模块
├── hybrid_astar_planner/     # Hybrid A* 全局规划器子模块
└── velocity_smoother_ema/    # EMA 速度平滑器子模块
```

- `teb_local_planner`：通过时间弹性带优化局部轨迹，生成避障速度指令。
- `hybrid_astar_planner`：提供考虑车辆运动学约束的 Hybrid A* 全局规划插件。
- `velocity_smoother_ema`：对线速度和角速度进行指数移动平均滤波，平滑速度指令。

## 安装依赖

在工作空间根目录克隆仓库及子模块：

```bash
git clone --recurse-submodules https://github.com/Johnson3176/moma_nav.git src/moma_nav
```

已有克隆补齐子模块：

```bash
git -C src/moma_nav submodule update --init --recursive
```

在已安装 ROS 1 并加载其环境的 Ubuntu 系统中安装其余依赖：

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-navigation ros-${ROS_DISTRO}-costmap-converter
rosdep update
rosdep install --from-paths src/moma_nav --ignore-src -r -y
```

首次使用 rosdep 时，先执行一次 `sudo rosdep init`。

子模块按本仓库记录的 Commit 检出。依赖升级时，先在子模块提交并推送修改，再在本仓库暂存对应子模块路径并提交、推送，更新固定版本；子模块的未提交修改不会随父仓库同步。

## 启动

在工作空间根目录编译并启动：

```bash
catkin_make
source devel/setup.bash
roslaunch moma_nav sentry_movebase.launch
```

默认启动 `move_base` 和 `project_body_to_map_2d`，可选启用速度平滑器。雷达、定位、地图与点云转激光扫描需单独启动。

## 输入与输出

以下话题名称以默认根命名空间启动为准。

| 输入 | 用途 |
| --- | --- |
| `/prior_map` | 静态地图，入口将 `/map` 重映射至此话题 |
| `/Odometry` | 里程计 |
| `/scan` | 代价地图激光扫描观测 |
| `/move_base_simple/goal` 或 `/move_base` action | 导航目标 |
| `map → body` TF | 定位系统提供的机器人三维位姿 |

| 输出 | 用途 |
| --- | --- |
| `/cmd_vel` | 默认由 move_base 发布的底盘速度指令 |
| `/raw_cmd_vel → /cmd_vel` | 启用平滑器后，move_base 发布原始速度，平滑器输出平滑速度 |
| `map → body_2d` TF | 二维位姿投影节点发布的平面位姿 |
| `/move_base` action 的反馈与结果 | 导航执行状态 |

坐标系与 TF 查询关系如下（箭头表示可通过 TF 链查询，不要求直接相连）：

```text
map  → base_link        全局代价地图使用的机器人位姿
odom → base_link        局部代价地图使用的机器人位姿
map  → body             二维投影节点输入，由外部定位系统提供
map  → body_2d          二维投影节点输出，由本包发布
map / odom → scan帧     将激光观测转换到相应代价地图坐标系
```

`scan帧` 为 `/scan` 消息的 `header.frame_id`；扫描源的 `sensor_frame` 当前配置为 `body_2d`，应核对其与实际扫描坐标系的对应关系。外部系统需提供其余 TF 连接，仅发布 `/Odometry` 消息不会自动建立 TF。

可用 `rosrun tf tf_echo map base_link`、`rosrun tf tf_echo odom base_link` 检查关键变换。

## 二维位姿投影

源码为 `moma_nav/src/project_body_to_map_2d.cpp`，节点及可执行程序名称为 `project_body_to_map_2d`。

节点读取 **map 坐标系下 body 的位姿**，将其投影到 **map 的 XY 平面**，广播 `map → body_2d`：

- 保留位置 x、y，将 z 设为零。
- 提取 yaw，构造 roll、pitch 为零的单位四元数。
- 动态 TF 保留输入时间戳，同一时间戳的样本只发布一次；完全静态的输入使用当前时间发布。
- 以 100 Hz 查询 TF；查询失败或输入无效时跳过发布，限频输出警告。

此节点只发布 TF，不转换点云、激光扫描或里程计消息，也不提供完整的导航 TF 链。

## 主要配置

参数文件位于 `moma_nav/param/`（相对于仓库根目录）。`sentry_movebase.launch` 加载以下参数：

| 文件 | 用途 |
| --- | --- |
| `costmap_common_params.yaml` | 机器人轮廓、障碍物观测和通用代价地图参数 |
| `global_costmap_params.yaml` | 全局代价地图 |
| `local_costmap_params.yaml` | 局部滚动代价地图 |
| `global_planner_params.yaml` | 全局路径规划 |
| `base_teb_local_planner_params.yaml` | TEB 局部规划与 costmap_converter 插件参数 |

`base_dwa_local_planner_params.yaml` 和 `costmap_converter_params.yaml` 为备选配置，默认不单独加载。切换 DWA 时需同时替换局部规划器与参数加载项；默认 TEB 文件已包含 costmap_converter 参数。
