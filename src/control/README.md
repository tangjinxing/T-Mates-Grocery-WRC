# WRC Control 总览与交接

更新日期：2026-08-07。本文是 `/home/lh/WRC/src/control` 的总入口；抓取细节见 `grasp_pipeline/README.md`，整体预设页面见 `initial_pose_console/README.md`，完整历史交接见 `HANDOFFE.md`。

## 1. 当前项目状态

工作区和环境：

```text
工作区：/home/lh/WRC
主代码：/home/lh/WRC/src
控制代码：/home/lh/WRC/src/control
Conda环境：hand_eye_calib
Python：3.10
系统：Ubuntu 22.04，aarch64
```

旧的 `/home/lh/WRC/camera calibration` 目录已删除，不要恢复或引用旧路径。标定数据、标定代码和网页现在位于 `/home/lh/WRC/src/hand_eye_calibration`。

硬件对应关系：

| 设备 | 地址/序列号 | 当前接口 |
|---|---|---|
| 左臂 | `169.254.128.18` | `/home/lh/robot_api/arm_api_new/realman_arm_api_api2.py` |
| 右臂 | `169.254.128.19` | 同上 |
| 头部 D435 | `344422070170` | `/home/lh/robot_api/camera_api/D435_rgb_depth.py` |
| 左手 D435 | `215322079194` | 同上 |
| 右手 D435 | `335522072306` | 同上 |
| 头部舵机 | `/dev/ttyUSB0`，9600 baud | `/home/lh/robot_api/head_api1/servo_api.py` |

## 2. 已完成内容

### 手眼标定

代码和数据根目录：`/home/lh/WRC/src/hand_eye_calibration`。

- ChArUco 板为 CC300-20-15，`DICT_5X5_100`，14×9 格点，最大 104 个内角点。
- 检测采用原始灰度、CLAHE、Gamma 三路候选，按角点质量和重投影质量选择，再用 `cornerSubPix` 亚像素细化。
- 左臂动态眼在手外：200 组，160 训练/40 验证，结果在 `datasets/dynamic_eye_to_hand_left/results/dynamic_160_40_five_repeats.json`。
- 右臂动态眼在手外：198 组独立有效样本，160 训练/38 验证，结果在 `datasets/dynamic_eye_to_hand_right/results/dynamic_160_38_five_repeats.json`。
- 左臂眼在手上：结果在 `datasets/eye_in_hand_left/results/eye_in_hand_20_5_five_repeats.json`。
- 右臂眼在手上：数据目录已建立，但当前未找到可用的 `results/*.json`，不能直接用于抓取。

### 控制与采集

- `grasp_pipeline/acquire_sample.py`：从统一 YAML 读取拍照预设，移动已勾选部件，采集 RGB、对齐深度和 `camera.json`。
- `grasp_pipeline/run_pose_target.py`：导入网页 6D 位姿，执行手眼变换、安全检查、预抓取、夹取和抓取后回退。
- `grasp_pipeline/test_native_gripper.py`：只测试因时原生夹爪，不移动机械臂。
- `initial_pose_console/app.py`：8010 整体预设姿态管理页面，管理头部、躯干、左臂和右臂字段。
- 统一预设文件：`grasp_pipeline/config/preset_poses.yaml`。
- 当前左臂眼在手上拍照预设：`shelf_upper_photo_eye_in_hand_left`，位姿来自统一 YAML，不再使用旧 JSON 拍照位姿。

## 3. 当前抓取流程

```text
读取小票/货架信息
→ 选择统一 YAML 预设拍照点
→ 采集 RGB/深度/camera.json
→ 外部网页估计 camera_T_object
→ 选择 eye_in_hand 或 eye_to_hand
→ 转换到机械臂基座坐标系
→ 逆解和安全检查
→ 最大打开夹爪
→ MoveJ_P 到预抓取点
→ 5% MoveL 到理论抓取点前10 mm
→ 2% MoveL 完成最后10 mm
→ speed=100、force=200 力控闭合
→ 回退前重新计算并检查路径
→ 基座+Z上升50 mm
→ 沿实际接近轴反向退出
→ 基座-Z下降50 mm
→ 低速 MoveJ_P 回到统一 YAML 拍照位
→ 后续再接导航和放置
```

当前 `run_pose_target.py` 参数以代码为准：

```text
预抓取 MoveJ_P：5%
预抓取到10 mm：5% MoveL
最后10 mm：2% MoveL
抓取后回退 MoveL：6%
回拍照位 MoveJ_P：6%
回退上升/下降：基座系 +Z/-Z，各50 mm
回退 MoveL 单段超时：90 s
TCP：法兰局部 +Z，130 mm
夹爪：speed=100，force=200
最大开爪确认：actpos >= 990
```

## 4. 未解决问题和限制

- 当前没有完整环境碰撞模型；货架、物体、夹爪和人员安全必须人工确认，实体急停必须可用。
- 回退使用机械臂基座系 `+Z/-Z` 作为竖直方向；如果现场坐标定义发生变化，必须重新做小距离方向验证。
- `test_base_minus_x_lift.py` 仅是历史的基座 `-X` 方向诊断脚本，不是当前正式回退方向。
- 右臂眼在手上结果尚未确认，右臂眼在手上抓取暂不能作为正式流程。
- 统一 YAML 目前已建立左侧上层眼在手外和眼在手上拍照预设；右侧或其他货架层预设需要后续补齐。
- 头部动态眼在手外计算使用实际 yaw/pitch；当前抓取执行前必须确保头部实际姿态与采图姿态一致。
- 网页位姿估计仍需人工上传 RGB、深度和相机内参；尚未与抓取终端自动串联。
- 抓取后目前只回到拍照位并保持夹爪闭合，不执行抬升后的导航、放置和松爪。
- SSH 没有图形显示时不能依赖 `cv2.imshow`；使用网页或保存图像检查。

## 5. 启动和测试命令

所有命令先执行：

```bash
conda activate hand_eye_calib
```

### 预设姿态管理页面

```bash
cd /home/lh/WRC/src/control/initial_pose_console
python app.py
```

访问 `http://主机IP:8010/`。主机 IP 变化时用 `hostname -I` 查看；8010 被占用时先查旧进程，不要重复启动。

### 采集拍照样本

```bash
cd "/home/lh/WRC/src/control/grasp_pipeline"

# 左臂眼在手上：左手D435 + 左臂
python acquire_sample.py --mode eye_in_hand_left

# 左臂眼在手外：头部D435 + 左臂
python acquire_sample.py --mode eye_to_hand_left
```

采集移动速度可通过命令行调整：

```bash
python acquire_sample.py --mode eye_in_hand_left --arm-speed 2 --lift-speed 5
```

`--arm-speed` 是机械臂 MoveJ_P 百分比，`--lift-speed` 是升降柱速度百分比；相机 `--fps` 只影响采集帧率，不影响机器人速度。

### 眼在手上网页

```bash
cd /home/lh/WRC/src/hand_eye_calibration/web_interface

# 左臂，8002
python start_left_eye_in_hand_web.py

# 右臂，8003；当前右臂结果尚未确认
python start_right_eye_in_hand_web.py
```

访问 `http://主机IP:8002/` 或 `http://主机IP:8003/`。相机和机械臂不能被其他网页或采集进程同时占用。

### 动态眼在手外网页

```bash
cd /home/lh/WRC/src/hand_eye_calibration/web_interface

# 左臂，8001
python start_left_eye_to_hand_web.py

# 右臂，8000
python dynamic_handeye_web/web_calibration_app.py \
  --profile eye_to_hand_right --host 0.0.0.0 --port 8000
```

### 动态模型计算

```bash
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code

python compute_dynamic_160_40.py \
  --scene ../datasets/dynamic_eye_to_hand_left

python compute_dynamic_160_40.py \
  --scene ../datasets/dynamic_eye_to_hand_right
```

完整 200 组使用 160/40；198 组使用 160/38。计算只读采集数据，结果写入场景 `results/`。

### 抓取执行

```bash
cd "/home/lh/WRC/src/control/grasp_pipeline"

python run_pose_target.py \
  --mode eye_in_hand \
  --arm left \
  --standoff 0.20 \
  --return-lift-mm 50 \
  --pose X_MM Y_MM Z_MM RX RY RZ
```

交互确认词必须按提示逐步输入：

```text
MOVE LEFT PREGRASP
APPROACH LEFT 10MM
FINAL GRASP LEFT
RETURN LEFT PHOTO
```

夹爪独立测试：

```bash
python test_native_gripper.py --speed 100 --force 200
python test_native_gripper.py --open-only --speed 100
```

## 6. 关键设计决定

- 机械臂末端位姿是“末端相对于该机械臂自身基座”，不是躯干或导航底盘世界坐标。
- 眼在手上使用手部相机和固定外部 ChArUco 板；眼在手外使用头部相机和动态 yaw/pitch 模型。
- TCP 偏移是法兰局部 `+Z` 方向 130 mm；接近轴是当前姿态旋转矩阵第三列，不是固定基座轴。
- 抓取后的竖直抬升使用机械臂基座系 `+Z`，当前上升/下降各 50 mm；反向退出使用当前实际末端姿态下的接近轴。
- 回退必须在抓取成功后根据实际末端状态重新规划，并在全部回退目标逆解通过后才允许执行。
- 每次抓取前夹爪主动最大打开并确认 `actpos >= 990`；未确认不移动机械臂。
- 动作全部保留人工确认词；程序不能替代现场碰撞确认或实体急停。
- 统一预设 YAML 只保存固定机器人状态；物体 6D、预抓取位姿和抓取位姿都是运行时计算，不写回预设。

## 7. 下一步任务

1. 完成左臂抓取后回拍照位的稳定性和携物碰撞验证。
2. 在不改变既有标定数据的前提下，串联回拍照位后的导航交接和放置姿态。
3. 补齐放置、松爪、回撤和异常中止后的恢复流程。
4. 完成右臂眼在手上网页采集后的计算与结果验证。
5. 为右侧货架和其他层级创建并验收统一 YAML 预设。
6. 将网页 6D 位姿输出与抓取终端通过文件或接口自动交接，减少手工复制。
7. 为每次抓取保存安全检查、实际位姿、夹爪状态和失败原因日志。

## 8. 常见错误及解决方式

### `Failed to fetch` 或网页无响应

确认服务进程仍在运行、访问 IP 是当前 `hostname -I` 中的地址、端口未被旧进程占用。8000/8001/8002/8003/8010 分别对应不同页面，不能混用页面用途。

### `Device or resource busy` / `xioctl ... busy`

同一台 D435 只能被一个进程打开。关闭旧网页、采集脚本和残留 Python 进程后再启动；不要同时让8010页面和 `acquire_sample.py` 连接同一相机。

### `qt.qpa.xcb` 或没有实时图像

SSH 的 `DISPLAY` 为空时没有 GUI 显示能力。使用局域网网页、本地查看保存的 RGB，或关闭 GUI 调用；这不是 ChArUco 检测算法本身的错误。

### 角点不足、图像过暗、曝光跳动

确认标定板完整进入画面、避免反光和过曝、等待相机预热；网页会在原始灰度/CLAHE/Gamma 候选中选择并做亚像素细化。不要为了凑数量保存质量明显不合格的样本。

### `逆解失败` 或 `单关节变化超过90°`

检查 6D 位姿是否为 `camera_T_object`、平移是否为毫米、旋转是否为 XYZ 弧度、模式和标定文件是否匹配。不要绕过安全检查；先在终端查看矩阵和目标位姿。

### `MoveL超过...s仍未到位`

先看误差是否持续下降。程序会主动停止运动；不要从中间位置直接重启完整抓取流程。先通过示教或安全回位流程返回拍照位。当前回退 MoveL 速度为 6%，单段超时90秒。

### 回到错误的拍照位

检查终端打印的 `拍照预设` 和 `拍照位姿文件`。当前必须是统一 YAML 的对应节点，例如 `shelf_upper_photo_eye_in_hand_left`；不要使用旧 `initial_photo_pose.json`。

### 夹爪未打开或夹取失败

检查 `enable_state=1`、`error=0`，确认打开后 `actpos >= 990`。因时原生夹爪使用 `speed=100、force=200` 作为当前可乐瓶测试参数；`force` 是等级参数，不直接等于牛顿。

### 端口 `address already in use`

先确认同端口旧服务是否仍在运行，再决定复用或停止旧服务；不要盲目启动多个实例。停止网页服务后，确认进程消失再重新启动。

## 9. 数据保护

- 不删除或覆盖 `/home/lh/WRC/src/hand_eye_calibration/datasets` 下已有数据、结果和 RGB 图像。
- `grasp_pipeline/runs/` 是重要采集数据；撤回样本进入 `rejected/`，不要手工删除记录。
- 修改预设前保留 `config/backups/` 中的 YAML 备份。
- 计算脚本只读数据并把新结果写入 `results/`。
- 任何代码或参数修改都要同步更新本总 README、`HANDOFFE.md` 和相关子目录 README。
