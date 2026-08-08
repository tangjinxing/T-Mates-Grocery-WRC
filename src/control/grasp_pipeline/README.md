# WRC 全流程抓取（grasp_pipeline）

本目录用于串联“回到统一 YAML 拍照位姿 → RGB/深度采集 → 外部网页6D位姿估计 → 手眼坐标转换 → 安全检查 → 预抓取、夹取、回退与固定点放置”。当前左臂因时原生夹爪流程已验证到抓取和回拍照位；独立放置脚本已实现躯干下降、MoveJ_P放置、松爪、回拍照位和躯干恢复，待实机分阶段验收。

## 当前文件

```text
grasp_pipeline/
├── acquire_sample.py       # 读取已保存拍照姿态，采集RGB/深度/camera.json
├── run_pose_target.py      # 导入6D位姿、坐标转换、安全检查、MoveJ_P运动
├── run_pose_target_6d.py   # 圆柱完整6DOF候选姿态实验版；尚未实机验收
├── place_object.py         # 左臂固定点放置、松爪、回拍照位和恢复躯干
├── config/
│   ├── preset_poses.yaml   # 头部、左臂、右臂固定预设状态的唯一注册表
│   ├── grasp_orientation.yaml # 货架正面接近方向及安全候选角范围
│   └── backups/            # 覆盖预设前的历史备份
├── runs/                   # 按模式和时间保存采集样本（重要数据）
└── README.md               # 本文档，后续流程变化统一维护在这里
```

## 固定预设状态

固定状态统一维护在：

```text
config/preset_poses.yaml
```

文件只保存头部 `yaw/pitch` 和左右臂基座系下的末端6D位姿，不保存底盘导航点，也不保存物体、预抓取或抓取等动态计算位姿。稳定命名采用：

```text
<场景>_<层级>_<阶段>_<模式或执行侧>
```

当前预设：

```text
level_1_left / level_1_right   # 第一层，等待记录躯干高度
level_2_left / level_2_right   # 第二层，已验收
level_3_left / level_3_right   # 第三层，等待记录躯干高度
```

旧的初始、小票、眼在手外和桌面放置预设已从当前YAML清理，但完整备份仍在
`config/backups/preset_poses_before_level_cleanup_20260808_173722.yaml`。因此
`place_object.py`在重新记录放置过渡点和终点之前不可执行。

每个预设均包含 `head/torso/left_arm/right_arm`，并通过 `apply_components` 明确本动作要移动哪些部件。参与执行的字段为 `null` 时禁止执行；未参与字段为 `null` 时保持不动，绝不能解释为零位。页面状态含义：`pending` 为尚未设置或全部参与字段无数据，`incomplete` 为部分参与字段缺失，`ready` 为全部参与字段可用。使用8010页面可以分别读取四个部件并更新执行范围；每次更新都会备份旧YAML并原子写入。页面允许在100～1350 mm安全范围内控制躯干高度，速度限制为1～20%，该功能依赖左臂控制器连接。

## 环境与硬件

```text
Conda环境：hand_eye_calib
左臂IP：169.254.128.18
右臂IP：169.254.128.19
头部D435：344422070170
左手D435：215322079194
右手D435：335522072306
```

相关依赖目录：

```text
/home/lh/robot_api/camera_api
/home/lh/robot_api/arm_api_new
/home/lh/WRC/src/hand_eye_calibration/datasets
/home/lh/WRC/src/control/grasp_pipeline/config/preset_poses.yaml
```

## 模式关系

| 模式 | 相机 | 可执行机械臂 | 当前状态 |
|---|---|---|---|
| `eye_to_hand` | 头部相机 | 左臂或右臂 | 均有动态标定结果 |
| `eye_in_hand` | 对应手部相机 | 相机所在机械臂 | 左臂可用；右臂缺少眼在手上结果 |

眼在手外：

```text
arm_base_T_object = arm_base_T_camera(yaw,pitch) × camera_T_object
```

眼在手上：

```text
arm_base_T_object = arm_base_T_gripper_at_capture
                  × gripper_T_camera
                  × camera_T_object
```

## 1. 采集RGB、深度和相机参数

进入目录：

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"
```

左臂眼在手外（头部相机）：

```bash
python acquire_sample.py --mode eye_to_hand_left
```

左臂眼在手上（左手相机）：

```bash
python acquire_sample.py --mode eye_in_hand_left --shelf-level 2
```

脚本从统一 YAML 读取当前模式对应的拍照预设，只连接并移动其 `apply_components` 中的部件；未参与部件保持不动。脚本显示全部目标后，必须输入完整确认词 `MOVE <预设名>`，随后以默认机械臂速度5%、升降柱速度5%依次到位，再启动相机采集。参与执行的字段缺失、躯干高度不在100～1350 mm或目标预设不存在时会拒绝运行。每组新样本保存在：

```text
runs/<模式>/<时间戳>/
├── rgb.png
├── depth.png       # uint16原始深度，已对齐到RGB
└── camera.json     # 网页需要的cam_K和毫米/单位depth_scale
```

运行前请在8010页面释放相机，并建议断开该页面的机械臂/头部连接，避免两个进程同时访问同一硬件。

## 眼在手上左臂测试

本节是当前左臂最小抓取验证的完整执行入口。运行前确认左手D435已经释放、左臂与躯干周围无人和障碍物、实体急停可用。建议先在8010页面释放相机并断开页面的机械臂连接，避免两个进程同时访问硬件。

> `run_pose_target.py` 仍是当前已验证的固定末端朝向主流程，不得被实验代码替换。
> `run_pose_target_6d.py` 是独立实验版。对于现场确认竖直摆放的圆柱饮料，视觉
> `+Y` 绿轴只用于朝上性和倾角诊断；控制向上轴固定为左臂基座 `+Z` 并对齐夹爪
> `-X`，避免视觉倾斜噪声导致接近路径产生额外Z分量。不使用会绕圆柱跳变的红蓝轴。
> 夹爪局部 `+Z` 接近方向来自
> `grasp_orientation.yaml` 保存的货架正面方向，并在绿轴法平面内生成
> `-20°、-10°、0°、+10°、+20°` 候选。程序对完整抓取和回退关键点逆解后，
> 选择最接近正面方向且关节变化、J5裕量更安全的候选。
> 首次只能使用 `--plan-only` 审查结果，不得直接执行运动。

完整6DOF第二层只读解算入口：

```bash
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm left \
  --shelf-level middle \
  --standoff 0.20 \
  --return-lift-mm 50 \
  --plan-only
```

示教只用于一次性提取货架正面方向，不是每次运行参数。顶层和第二层货架当前
共用 `shelf_front_left`；若以后货架朝向不同，可新增方向配置。物体坐标轴必须
保证绿色+Y始终朝上，否则禁止执行实验版。

实验版可用 `--up-axis-policy base-z`（默认，强制基座竖直）与
`--up-axis-policy measured`（精确跟随视觉绿色轴）进行只读A/B对比。首次使用
`measured` 必须保留 `--plan-only`；测量倾角超过配置上限时程序直接拒绝。

### 第一步：自动到达左臂拍照位并采集

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python acquire_sample.py --mode eye_in_hand_left
```

脚本读取统一预设 `level_2_left`。当前该预设只执行 `torso + left_arm`，头部和右臂保持不动。核对终端显示的目标后，现场输入：

```text
MOVE level_2_left
```

脚本自动到达拍照位，然后使用左手D435保存：

```text
runs/eye_in_hand_left/<时间戳>/rgb.png
runs/eye_in_hand_left/<时间戳>/depth.png
runs/eye_in_hand_left/<时间戳>/camera.json
```

### 第二步：网页估计6D位姿

将同一时间戳目录中的 `rgb.png + depth.png + camera.json` 上传到位姿估计网页，取得：

```text
X_MM Y_MM Z_MM RX RY RZ
```

其含义必须是 `camera_T_object`；平移单位为毫米，旋转为XYZ欧拉角弧度。

### 第三步：预抓取、10 mm接近与夹取

将网页输出的六个数替换到下面命令：

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python run_pose_target.py \
  --mode eye_in_hand \
  --arm left \
  --standoff 0.20 \
  --return-lift-mm 50 \
  --pose X_MM Y_MM Z_MM RX RY RZ
```

也可以省略 `--pose`，启动后直接粘贴位姿估计网页输出：

```bash
python run_pose_target.py \
  --mode eye_in_hand \
  --arm left \
  --standoff 0.20 \
  --return-lift-mm 50
```

程序支持单行空格、单行逗号、方括号，以及网页复制出的“每个数单独一行、逗号也可单独一行”格式。读取满6个数后自动进入坐标转换和安全检查，不需要输入额外结束符。

安全检查通过后，第一阶段现场确认词：

```text
MOVE LEFT PREGRASP
```

左臂以10%速度 `MoveJ_P` 到预抓取点并验证实际位置误差。现场检查夹爪朝向、物体位置和直线路径后，第二阶段确认词：

```text
APPROACH LEFT 10MM
```

左臂以5%速度发送非阻塞 `MoveL`，沿法兰局部 `+Z` 在基座系中的实际方向接近，先让夹爪中心停在理论抓取点前10 mm。程序每0.2秒主动读取实际位姿；位置误差不超过8 mm、姿态误差不超过2°并连续满足3次后判定到位。超过60秒、机械臂报错或连续4秒无至少1 mm进展时，程序主动发送 `move_stop()` 并退出。

现场确认两指位于同一物体两侧且最后10 mm路径安全后，输入第三阶段确认词：

```text
FINAL GRASP LEFT
```

程序以4%速度非阻塞 `MoveL` 前进最后10 mm（超时15秒），随后使用因时原生夹爪执行 `speed=100、force=200` 的 `gripper_pick_keep` 持续力控夹取。程序以夹爪在线、使能、无错误、发生明显闭合、开度稳定且 `mode=6` 作为成功条件。`current_force` 仅用于监控和记录，不再使用硬编码 `>=50` 门槛。夹爪接近完全闭合、仍接近全开、非mode=6或15秒未稳定会报错，并自动发送重新打开命令。

### 第四步：抓取后回退

夹取成功后，程序重新读取实际抓取位姿并重新规划：

```text
基座+Z上升50 mm
→ 沿实际接近轴反向MoveL退回预抓取区域
→ 基座-Z下降50 mm
→ 低速MoveJ_P回到统一YAML拍照位姿
```

回退前会重新执行工作范围、移动距离、机械臂错误、逆解和关节变化检查。通过后输入：

```text
RETURN LEFT PHOTO
```

当前代码回退 `MoveL` 速度为12%，单段超时90秒；回拍照位 `MoveJ_P` 速度为12%。回到拍照位后保持夹爪闭合，不自动放置或松爪。

### 第五步：导航到桌面后执行独立放置

导航组将机器人移动到放置桌面前，并确认左臂仍位于眼在手上拍照姿态、躯干仍处于拍照高度且夹爪稳定夹持物体后，运行：

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python place_object.py
```

脚本读取：

```text
过渡目标：table_place_transition_left
最终放置目标：table_place_final_left
拍照目标：shelf_upper_photo_eye_in_hand_left
```

历史预设 `table_place_pre_left` 保留不删除，新脚本不再默认读取它。默认流程和确认词为：

```text
启动状态检查：左臂在拍照位、躯干在拍照高度、夹爪正在夹持
LOWER TORSO 400       → 10%下降躯干至400 mm
MOVE LEFT TRANSITION  → 5% MoveJ_P到中间过渡位
MOVE LEFT PLACE       → 5% MoveJ_P从过渡位到最终放置位
RELEASE LEFT OBJECT   → 确认物体由桌面承托后打开夹爪
RETURN LEFT TRANSITION → 5% MoveJ_P原路回过渡位
RETURN LEFT PHOTO     → 5% MoveJ_P从过渡位回拍照姿态
RAISE TORSO 1348      → 10%恢复拍照躯干高度
```

过渡目标使用 `table_place_transition_left` 的完整6DOF。最终放置目标只使用 `table_place_final_left.left_arm.pose_6d` 的 `x/y/z`，其记录的 `rx/ry/rz` 不参与执行；实际最终末端旋转固定使用 `shelf_upper_photo_eye_in_hand_left` 的拍照姿态。过渡点来自只读IK候选搜索，正反四段端点IK最大单关节变化约65.1°，但尚未证明环境无碰撞。每个运动阶段发送指令前都会重新读取实际状态，并检查目标数值、工作范围、机械臂错误、移动距离、逆解及单关节最大变化。升降柱默认速度为10%，检查允许范围为100～1350 mm、到位容差为±10 mm；机械臂到位容差为10 mm和3°。程序没有环境模型，不能自动检查 `MoveJ_P` 中间轨迹碰撞，首次必须空载或由现场人员低速观察验证。任一检查失败或确认词不匹配，后续阶段不会执行。

## 2. 外部网页估计6D位姿

向现有位姿估计网页上传同一样本中的：

```text
rgb.png + depth.png + camera.json
```

网页输出约定：

```text
[x_mm, y_mm, z_mm, rx, ry, rz]
```

- 平移单位：毫米
- 旋转单位：弧度
- 旋转表示：XYZ欧拉角
- 含义：`camera_T_object`

## 3. 坐标转换与预抓取测试

眼在手上左臂示例：

```bash
python run_pose_target.py \
  --mode eye_in_hand \
  --arm left \
  --standoff 0.20 \
  --return-lift-mm 50 \
  --pose X_MM Y_MM Z_MM RX RY RZ
```

眼在手外左臂示例：

```bash
python run_pose_target.py \
  --mode eye_to_hand \
  --arm left \
  --standoff 0.20 \
  --return-lift-mm 50 \
  --pose X_MM Y_MM Z_MM RX RY RZ
```

脚本将：

1. 读取对应手眼标定结果。
2. 将物体位姿转换到执行机械臂基座系。
3. 采用TCP平移近似：夹爪中心相对法兰沿末端局部 `+Z` 偏移130 mm，并通过当前末端旋转矩阵转换到基座系。
4. 保持当前末端姿态，生成预抓取法兰位姿和距理论抓取点10 mm的停止位姿。
5. 检查矩阵、深度、工作范围、移动距离、机械臂/夹爪错误、两个目标的逆解和关节变化。
6. 输入 `MOVE LEFT PREGRASP` 后，确认夹爪打开并以10%速度执行 `MoveJ_P` 到预抓取点。
7. 验证预抓取实际位置误差不超过15 mm；现场检查后输入 `APPROACH LEFT 10MM`。
8. 以5%速度非阻塞执行 `MoveL`，由Python主动监控位姿、错误、60秒超时和停滞，沿末端局部 `+Z` 的实际方向停在夹爪中心距理论抓取点10 mm处。
9. 输入 `FINAL GRASP LEFT` 后，以4%速度完成最后10 mm直线接近，再以 `speed=100、force=200` 持续力控闭合夹爪并检查 `mode=6`。
10. 输入 `RETURN LEFT PHOTO` 后，执行+Z上升50 mm、反向退出、-Z下降50 mm和MoveJ_P回拍照位。

输入其他内容会取消，不发送运动命令。

## 当前安全边界

- 当前已实现最大开爪、预抓取、10 mm接近、力控闭合和安全回退；回退成功后保持夹爪闭合。
- 首次测试停止距离限制为0.10～0.30 m，推荐0.20 m。
- 单次笛卡尔位移上限0.60 m。
- 单关节逆解变化超过90°时拒绝执行。
- 程序没有环境碰撞模型；执行前必须人工确认周围无人且无障碍物，实体急停必须可用。
- `MoveJ_P`中间轨迹不是直线，正式抓取后续采用“MoveJ_P到预抓取点 + MoveL短距离接近”。
- 拍照后如果底盘、物体、相机或机械臂拍照位姿变化，原6D结果立即作废，必须重新采集和估计。
- 眼在手外当前使用保存的头部yaw/pitch计算动态外参，执行前必须保证头部实际到位。

## 左臂因时原生夹爪

当前左臂安装的是因时原生夹爪，由瑞尔曼机械臂控制器直接通过原生夹爪 API 控制。不要使用大寰 `control_gripper_dh` 或钧舵 `control_gripper_jd` 的 Modbus 接口。

独立力控夹取测试（不会移动机械臂）：

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python test_native_gripper.py --speed 100 --force 200
```

把有外部支撑的物品放到两指之间、手指离开后，输入 `CLOSE GRIPPER` 执行力控闭合。人工检查牢固度后输入 `OPEN GRIPPER` 重新打开。如果程序中断或只需释放夹爪：

```bash
python test_native_gripper.py --open-only --speed 100
```

打开前必须确认物品有支撑，不会因释放而掉落。测试脚本不会控制或移动机械臂。

接口位于：

```text
/home/lh/robot_api/arm_api_new/realman_arm_api_api2.py
```

主要接口：

```python
# 设置有效行程，范围0～1000
client.configure_gripper_range(0, 1000)

# 完全打开
client.gripper_release(speed=100, block=False, timeout=1)

# 力控夹取；force允许范围50～1000
client.gripper_pick_keep(speed=100, force=200, block=False, timeout=1)

# 持续力控夹取
client.gripper_pick_keep(speed=100, force=50, block=False, timeout=1)

# 移动到指定开度，position允许范围1～1000
client.gripper_move_to(500, block=False, timeout=1)

# 读取实际状态
state = client.get_gripper_state()
```

`get_gripper_state()` 主要字段：

```text
enable_state   # 1表示已使能
status         # 当前状态
error          # 0表示无夹爪错误
mode           # 当前工作模式
current_force  # 当前力值
temperature    # 夹爪温度
actpos         # 实际开度，约1000为全开
```

2026-08-06 已在左臂 `169.254.128.18` 完成实机验证：

```text
configure_gripper_range(0, 1000)   成功
gripper_move_to(500)               成功，实际位置534
gripper_release(speed=100)         成功，最终实际位置999
最终 enable_state=1、error=0、temperature=40°C
```

当前验证中，非阻塞调用比阻塞调用更稳定。非阻塞命令发出后必须等待并轮询 `get_gripper_state()`，不能只根据命令返回值判断动作完成。当前抓取顺序：

```text
检查 enable_state=1 且 error=0
→ 配置行程0～1000
→ 低速打开夹爪
→ 等待并确认实际开度
→ MoveJ_P到预抓取位姿
→ 5% MoveL到抓取点前10 mm并人工确认
→ 4% MoveL完成最后10 mm
→ 执行gripper_pick（speed=100、force=200，当前可乐瓶实测参数）
→ 轮询状态、实际位置和力值确认夹取结果
→ 基座+Z上升50 mm
→ 12% MoveL反向退出
→ 基座-Z下降50 mm
→ 12% MoveJ_P回拍照位
```

当前正式可乐瓶测试使用 `speed=100、force=200`，后续换物品时应先做独立夹爪测试再调整力控等级。夹爪约130 mm的法兰到夹持中心偏移在目标位姿计算中按末端局部 `+Z` 处理，并随末端旋转转换到机械臂基座系。当前只使用TCP平移，暂不加入额外TCP旋转。

## 当前状态与后续计划

已完成统一 YAML 拍照位读取、左臂眼在手上最小抓取、夹爪力控闭合和抓取后回拍照位。已新增独立左臂固定点放置脚本，待完成空载和夹物分阶段实机验收。后续重点是导航与放置的状态交接、放置失败恢复，以及右臂眼在手上结果验证。

## 维护约定

后续新增脚本、参数、矩阵方向、启动命令或流程变化时，必须同步更新本README。`runs/` 是重要实验数据，不得批量删除、覆盖或重命名；新增结果使用新的时间戳目录。

conda activate hand_eye_calib
cd /home/lh/WRC/src/control/initial_pose_console
python app.py
http://169.254.128.40:8010/

左臂夹爪打开：python test_native_gripper.py --open-only --speed 100

左臂夹爪闭合：python test_native_gripper.py \
  --speed 100 \
  --force 200

左臂夹爪使能：
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python - <<'PY'
import sys
import time

sys.path.insert(0, "/home/lh/robot_api/arm_api_new")
from realman_arm_api_api2 import RealmanArmClient

client = RealmanArmClient(
    ip="169.254.128.18",
    model="left",
    auto_connect=False,
)

try:
    client.connect()

    print("配置夹爪行程 0～1000……")
    client.configure_gripper_range(0, 1000)
    time.sleep(1)

    print("发送夹爪打开命令……")
    client.gripper_release(speed=100, block=True, timeout=10)
    time.sleep(1)

    state = client.get_gripper_state()
    print("夹爪状态:", state)

    if state.enable_state == 1 and state.status == 1 and state.error == 0:
        print("夹爪已成功使能并在线")
    else:
        print("夹爪仍未使能，请检查工具端24V、通信线及控制器夹爪配置")
finally:
    client.disconnect()
PY
右臂夹爪使能：
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python - <<'PY'
import sys
import time

sys.path.insert(0, "/home/lh/robot_api/arm_api_new")
from realman_arm_api_api2 import RealmanArmClient

client = RealmanArmClient(
    ip="169.254.128.19",
    model="right",
    auto_connect=False,
)

try:
    client.connect()

    print("右臂连接成功")
    print("配置右臂夹爪行程 0～1000……")
    client.configure_gripper_range(0, 1000)
    time.sleep(1)

    print("发送右臂夹爪打开命令……")
    client.gripper_release(speed=100, block=True, timeout=10)
    time.sleep(1)

    state = client.get_gripper_state()
    print("右臂夹爪状态:", state)

    if state.enable_state == 1 and state.status == 1 and state.error == 0:
        print("右臂夹爪已成功使能并在线")
    else:
        print(
            "右臂夹爪仍未使能，请检查工具端24V、"
            "通信线及控制器夹爪配置"
        )
finally:
    client.disconnect()
PY

右臂夹爪打开：
python test_native_gripper.py   --arm right   --open-only   --speed 100

种类抓取测试命令：

第二层拍照命令：
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python acquire_sample.py \
  --mode eye_in_hand_left \
  --shelf-level middle
第二层抓取命令：
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python run_pose_target.py \
  --mode eye_in_hand \
  --arm left \
  --shelf-level middle \
  --standoff 0.20 \
  --return-lift-mm 50



新指令：
右臂第一层：
拍照：
python acquire_sample.py   --mode eye_in_hand_right   --shelf-level 1
抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm right \
  --shelf-level 1 \
  --preset level_1_right \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50


右臂第二层：
拍照：
python acquire_sample.py   --mode eye_in_hand_right   --shelf-level 2
抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm right \
  --shelf-level 2 \
  --preset level_2_right \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50

左臂第一层：
拍照：
python acquire_sample.py   --mode eye_in_hand_left   --shelf-level 1
抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm left \
  --shelf-level 1 \
  --preset level_1_left \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50

左臂第二层：
拍照：
python acquire_sample.py   --mode eye_in_hand_left   --shelf-level 2
抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm left \
  --shelf-level 2 \
  --preset level_2_left \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50



夹爪松开：
python test_native_gripper.py   --arm left   --open-only   --speed 100
