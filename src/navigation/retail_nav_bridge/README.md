# retail_nav_bridge

商超「零售服务岗」建图导航统一门面：对上暴露 `/retail_nav/v1/*`，对下可接 Mock 或 TX-S2。

## 包结构

```text
ros2_ws/src/
  retail_nav_msgs/       # msg / srv / action
  retail_nav_bridge/     # 状态机 + Facade + Mock/S2 Adapter + ROS 节点
```

## 编译（在有 ROS2 的机器上）

```bash
cd ros2_ws
rosdep install --from-paths src -y --ignore-src   # 如需要
colcon build --packages-select server slam move retail_nav_msgs retail_nav_bridge
source install/setup.bash
```

## 启动 Mock（联调中控）

```bash
ros2 launch retail_nav_bridge mock_bridge.launch.py
```

## 中控调用示例

```bash
# 自检
ros2 service call /retail_nav/v1/Init retail_nav_msgs/srv/Init {}

# 加载地图并定位到起点
ros2 service call /retail_nav/v1/LoadMap retail_nav_msgs/srv/LoadMap "{map_name: 'retail_default'}"
ros2 service call /retail_nav/v1/Relocate retail_nav_msgs/srv/Relocate "{station_id: 'start'}"

# 导航到交付台
ros2 action send_goal /retail_nav/v1/GoToStation retail_nav_msgs/action/GoToStation \
  "{station_id: 'delivery_desk', request_id: 't1', max_speed: 0.4, timeout_sec: 60.0}" --feedback

# 订阅状态 / 事件
ros2 topic echo /retail_nav/v1/status
ros2 topic echo /retail_nav/v1/event
```

## Agent HTTP 网关

`mock_bridge.launch.py` 会同时启动 HTTP 网关，默认监听 `0.0.0.0:8081`。网关将
`target_id` 原样转换为 `GoToStation.station_id`，因此该编号必须存在于
`config/retail_stations.yaml`。

`GET /navigation/health` 会综合检查 ROS Action、状态话题和 `/retail_nav/v1/GetHealth`。
只有状态允许导航，且 `runtime_ready`、`navigation_ready`、
`localization_ready`、`motion_ready` 全部为 `true` 时才返回 `READY`；
建图能力 `mapping_ready` 不作为执行导航任务的必要条件。

```bash
# bridge 完成 Init、LoadMap 和 Relocate 后返回 READY
curl http://127.0.0.1:8081/navigation/health

curl -X POST http://127.0.0.1:8081/navigation/navigate \
  -H 'Content-Type: application/json' \
  -H 'Idempotency-Key: nav-test-001' \
  -d '{"target_id":"H1_F_L1_C01"}'
```

导航真正到达后返回 `{"status":"SUCCEEDED"}`。请求必须携带
`Idempotency-Key`；相同键的重试返回首次执行结果，不会重复执行物理动作。监听地址和端口可在
启动时修改：

```bash
ros2 launch retail_nav_bridge mock_bridge.launch.py \
  http_host:=0.0.0.0 http_port:=8081
```

商品货位解析配置位于 `config/product_slot_navigation.yaml`。每个货架面的
`levels.L1`～`L5` 分别记录该层总列数，网关使用“商品列中心离哪个四等分点最近”的规则选择
导航点。因此同一个 `Cxx` 在不同层可能映射到不同分段。配置还将 Agent 使用的
`delivery_place`、`replenishment_pickup`、`task_boundary` 分别映射到现有业务点。
各层列数必须与商品摆放表一致，否则网关会拒绝该层超出范围的列号。

## 无 ROS 单元测试（本机可跑）

```bash
cd ros2_ws/src/retail_nav_bridge
pip install PyYAML pytest
pytest test -q
```

## Adapter

| adapter 参数 | 说明 |
|--------------|------|
| `mock`（默认） | 匀速仿真到点，供中控联调 |
| `s2` | 天机 TX-S2：对接 `server/*`、`slam/*`、`move/*` |
| `woosh` | 华汇悟时底盘：对接 `woosh_robot/robot/ExecTask` 等 |
| `lh` | 云迹 WATER（非华汇默认真机）：`lh_chassis_bridge` `/chassis/*` |

```bash
# 天机真机
ros2 launch retail_nav_bridge mock_bridge.launch.py adapter:=s2
```

### 华汇机器人（悟时 Woosh 底盘）

华汇 `ts\robot` 已安装悟时 deb / `chassis_ros`。在同一工作空间加入天机的
`retail_nav_msgs`、`retail_nav_bridge` 后编译：

```bash
cd ~/robot   # 或 ts\robot
colcon build --packages-select retail_nav_msgs retail_nav_bridge
source install/setup.bash

# 确保悟时 robot-agent 与 ROS_DOMAIN_ID 已按 build.md 配置

ros2 launch retail_nav_bridge woosh_bridge.launch.py
```

启动前必须先填完 `config/retail_stations.yaml` 中的 `vendor_mark`：它是悟时
地图的真实储位号（例如 `A1`），不是业务站点 ID。模板值为空时桥接会拒绝导航，
不会把 `shelf_group_a_front` 之类的天玑站点名误发给底盘。

悟时默认要求显式选择厂家地图，并以“记录当前位置”的安全方式初始化；其中
`<woosh_map_name>` 必须替换为厂家地图名：

```bash
ros2 service call /retail_nav/v1/LoadMap retail_nav_msgs/srv/LoadMap \
  "{map_name: '<woosh_map_name>'}"
ros2 service call /retail_nav/v1/Relocate retail_nav_msgs/srv/Relocate '{}'
```

只有当 `retail_stations.yaml` 的坐标已在悟时地图坐标系完成标定时，才能设置
`woosh_allow_pose_relocation:=true` 并通过带坐标或 `station_id` 的 `Relocate`
写入定位；迁移来的天玑坐标不能直接使用。

Woosh 映射：

- `LoadMap` → `woosh_robot/robot/SwitchMap`
- `Relocate` → `woosh_robot/robot/InitRobot`（设定位姿）
- `GoToStation` → `ExecTask`，`mark_no` 取 `retail_stations.yaml` 中必填的
  `vendor_mark`（悟时地图储位号，如 `A1`）
- 位姿 ← `woosh_robot/robot/PoseSpeed`；任务状态 ← `TaskProc` / `OperationState`
- **不支持**坐标直达导航（无 S2 式 `CreateTaskByPose`）；货架点须在悟时地图里建同名储位
- 建图请在厂家工具完成

`woosh_allow_station_id_as_mark:=true` 仅适用于业务 `station_id` 与厂家 `mark_no`
完全一致的地图；默认关闭。导航是否到达只以本次 `ExecTask` Action 的结果为准，
不会以其他任务状态或模板坐标提前判定到达。

Agent HTTP 与天机相同：`GET /navigation/health`、`POST /navigation/navigate`。

### 云迹 WATER（`adapter:=lh`，非悟时华汇车可忽略）

华汇工程若使用 `chassis_water/lh_chassis_bridge`，见 `lh` 适配器；与悟时 `woosh` 二选一。

S2 映射：`StartMapping`→`server/StartMapping`，`StopMapping`→`server/StopMapping`，
`LoadMap`→`SwitchMap`/`slam/SetMap`，`Relocate`→`slam/Relocation`，
`GoToStation`→默认 `CreateTaskByPose`（YAML 的 x/y/yaw，适配 HMI **导航节点**）；
可选 `s2_station_nav_mode:=id` 走 `CreateTaskById`（需 **站台节点**），
`GoToPose`→`CreateTaskByPose`，`MoveRelative`→`move/MoveRelativeCmd`，
位姿←`slam/LocationData`（`/server/State` 本机常缺失，健康检查可回退）。

点位配置：`config/retail_stations.yaml`（把导航节点坐标填进对应 `station_id`）。

当前模板共 20 个点位：`start`、`restock_desk`、`delivery_desk`、
`judge_zone_1`，以及两组货架各自正反面的 4 个分段点。货架分段命名规则为：

- 货架组 A 正面：`shelf_group_a_front`、`shelf_group_a_front_2`～`_4`
- 货架组 A 反面：`shelf_group_a_back`、`shelf_group_a_back_2`～`_4`
- 货架组 B 正面：`shelf_group_b_front`、`shelf_group_b_front_2`～`_4`
- 货架组 B 反面：`shelf_group_b_back`、`shelf_group_b_back_2`～`_4`

无数字后缀表示面对该货架面的第 1 分段；四个点均按面对货架时从左到右排列。模板中标记
`TODO` 的坐标和厂家 ID 必须在真机使用前完成标定。四个水平分段负责覆盖货架宽度，垂直层位
仍由控制模块的 `shelf_level` 处理。

### 厂商原生地图导出

`ExportVendorMapPackage` 只在 `adapter:=s2` 下可用。它调用厂家
`server/GetMapInfo`，将当前 HMI 建图并编辑后的地图原样导出到：

```text
/home/nvidia/kai/TianJi/ros2_ws/src/retail_nav_bridge/config/map_packages/
  <map_name>_<UTC时间戳>/
    manifest.yaml
    map.bmap     # 必需：厂家原生主地图文件
    map.png      # 厂家提供时导出
    map.xml      # 厂家提供时导出
    map.pcd      # 厂家提供时导出
```

```bash
ros2 service call /retail_nav/v1/ExportVendorMapPackage \
  retail_nav_msgs/srv/ExportVendorMapPackage \
  "{map_id: '', map_name: 'retail_default', notes: 'HMI 已编辑导航点'}"
```

厂家文档没有单独的“导出导航点”接口，也没有公开 `.bmap` 的内部格式；
因此桥接节点不会尝试解析或伪造点位。请以 `.bmap` 作为包含 HMI 编辑状态
的原生导出物，并在真机上用重新导入/切图验证导航节点是否保留。

### 比赛业务点位同步（与 LoadMap、厂家地图导出分离）

`retail_stations.yaml` 是本项目的业务 `station_id → 坐标` 目录，和厂家
原生地图包不同。`SyncStations` 仍只从含有 `stations.yaml` 的业务点位包
（或直接的 YAML 文件）显式写回 `stations_file`；不会在 `LoadMap` 或
`ExportVendorMapPackage` 时自动执行。

```bash
ros2 service call /retail_nav/v1/SyncStations retail_nav_msgs/srv/SyncStations \
  "{package_path: '/path/to/business-stations.yaml'}"
```

```bash
# 真机：按导航点坐标导航（显式指定 stations_file 以免读到旧 install 配置）
ros2 launch retail_nav_bridge mock_bridge.launch.py adapter:=s2 \
  stations_file:=$(ros2 pkg prefix retail_nav_bridge)/share/retail_nav_bridge/config/retail_stations.yaml
```
