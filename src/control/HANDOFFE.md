# WRC 手眼标定与抓取流程交接文档

> 最新状态：2026-08-07。先阅读 `/home/lh/WRC/src/control/README.md` 总览，再阅读本文件；下面内容以当前代码和当前统一 YAML 为准。

## 当前状态速览

### 当前项目状态

- 代码根目录：`/home/lh/WRC/src`；控制入口：`/home/lh/WRC/src/control`。
- Conda：`hand_eye_calib`；Ubuntu 22.04、aarch64、Python 3.10。
- 旧 `/home/lh/WRC/camera calibration` 已删除，不要使用旧路径。
- 当前已验证左臂眼在手上“采集→网页6D输入→抓取→夹爪闭合→安全回退→回拍照位”流程。

### 已完成内容

- ChArUco CC300-20-15 检测、三路图像增强候选和亚像素细化。
- 左/右动态眼在手外数据和动态模型计算；左臂 200 组、右臂独立有效 198 组。
- 左臂眼在手上结果文件已生成；右臂使用 `eye_in_hand_right_v3` DANIILIDIS结果。
- 统一预设 `config/preset_poses.yaml` 已接入 `acquire_sample.py` 和 `run_pose_target.py`。
- 三层左右眼在手上拍照位统一读取 `level_1/2/3_left/right`，不再读取旧拍照位名称。
- 因时原生夹爪已实机验证，当前可乐测试使用 `speed=100、force=200`。

### 未解决问题

- 无完整环境碰撞模型，所有货架/物体/人员碰撞仍需人工确认。
- 抓取后已回到拍照位，但导航交接、放置姿态、松爪和异常恢复尚未串完。
- 右臂眼在手上结果尚未完成确认。
- 统一 YAML 当前主要有左侧上层眼在手外和眼在手上拍照预设；其他侧/层需要补充。
- 网页 6D 位姿仍需人工上传并复制到终端。

### 下一步任务

1. 验证携物回拍照位后与导航组的交接。
2. 添加放置位姿、放置前安全检查、松爪、回撤和失败恢复。
3. 完成右臂眼在手上计算及实机验证。
4. 补充右侧和其他货架层的统一 YAML 预设。
5. 保存每次抓取的目标矩阵、实际末端位姿、夹爪状态和失败原因。

### 关键设计决定

- 机械臂末端位姿是末端相对于该机械臂自身基座，不是底盘或躯干世界坐标。
- TCP 为法兰局部 `+Z` 方向 130 mm；接近轴是当前末端旋转矩阵第三列。
- 抓取后竖直方向当前定义为机械臂基座系 `+Z/-Z`，上升和下降各 50 mm；不能把 TCP 局部 `+Z` 直接当作竖直方向。
- 当前正式回退顺序：`+Z上升→沿接近轴反向退出→-Z下降→MoveJ_P回拍照位`。
- 夹取前夹爪必须主动打开并确认 `actpos >= 990`；所有阶段保留人工确认词。
- 固定状态只保存到 `config/preset_poses.yaml`；物体位姿、预抓取和抓取位姿运行时计算，不写回预设。

### 常见错误处理

- `Failed to fetch`：检查网页进程、当前主机 IP、端口和旧服务；不要重复启动同端口服务。
- `Device or resource busy`：同一相机被其他网页或脚本占用，先释放相机和残留进程。
- `MoveL超过...s`：程序会停止运动；不要从中途重跑，先安全回位。当前回退 MoveL 为 6%、单段超时 90 秒。
- 回错拍照位：检查终端打印的 `拍照预设`，必须是统一 YAML 节点，不是旧 JSON。
- 逆解/关节跳变拒绝：检查模式、单位、`camera_T_object`、欧拉角顺序和 TCP，不要绕过检查。
- `qt.qpa.xcb`：SSH 无 DISPLAY，使用网页或保存图像检查，不依赖 GUI。

## 1. 项目位置与环境

- 工作区：`/home/lh/WRC`
- 当前主代码目录：`/home/lh/WRC/src`
- Conda 环境：`hand_eye_calib`
- Python：3.10（Ubuntu 22.04，aarch64）
- 推荐启动环境：

```bash
conda activate hand_eye_calib
```

旧目录 `camera calibration` 已按此前整理流程删除；后续只使用 `src` 下内容，避免引用旧路径。

## 2. 当前目录结构

```text
/home/lh/WRC/src/
├── hand_eye_calibration/
│   ├── calibration_code/
│   ├── web_interface/
│   └── datasets/
└── control/
    ├── HANDOFFE.md
    ├── grasp_pipeline/
    ├── initial_pose_console/
    └── ...
```

### 2.1 手眼标定

`hand_eye_calibration` 保存标定代码、网页和数据集。主要标定模式：

- 眼在手外：头部相机固定或按节点采集 yaw/pitch，配合左/右臂末端位姿。
- 眼在手上：相机刚性安装在机械臂末端，标定板固定在外部。

主要数据目录：

```text
/home/lh/WRC/src/hand_eye_calibration/datasets/eye_in_hand_left
/home/lh/WRC/src/hand_eye_calibration/datasets/eye_in_hand_right
/home/lh/WRC/src/hand_eye_calibration/datasets/dynamic_eye_to_hand_left
/home/lh/WRC/src/hand_eye_calibration/datasets/dynamic_eye_to_hand_right
```

计算结果通常位于对应目录的 `results/` 中。

### 2.2 当前抓取流程代码

当前实际使用目录：

```text
/home/lh/WRC/src/control/grasp_pipeline/
├── acquire_sample.py
├── run_pose_target.py
├── test_native_gripper.py
├── config/
└── README.md
```

核心文件是 `run_pose_target.py`，负责把网页输出的 6D 位姿、当前机器人状态和手眼标定结果转换为机械臂目标，并执行安全检查、预抓取、直线接近和夹爪闭合。

## 3. 硬件与网络信息

### 3.1 机械臂

- 左臂 IP：`169.254.128.18`
- 右臂 IP：`169.254.128.19`
- Realman SDK API 版本已验证：`1.1.1`
- 接口文件：`/home/lh/robot_api/arm_api_new/realman_arm_api_api2.py`
- 机械臂末端位姿均为“末端/法兰相对于该机械臂自身基座”的位姿，不是躯干或底盘世界坐标。

执行机械臂动作前必须进行现场安全确认。程序没有完整环境碰撞模型，不能自动判断货架、物品、人员和夹爪碰撞。

### 3.2 RealSense D435

- 头部相机：序列号 `344422070170`
- 左手相机：序列号 `215322079194`
- 右手相机：序列号 `335522072306`

不同分辨率必须使用对应内参。已保存过工厂内参文件时，必须确认当前采图分辨率与内参文件一致，例如 640×480 和 1280×720 不可混用内参。

### 3.3 夹爪

- 夹爪类型：因时（通过 Realman 原生夹爪接口控制）
- 原生接口：
  - `configure_gripper_range`
  - `gripper_release`
  - `gripper_pick`
  - `gripper_pick_keep`
  - `gripper_move_to`
  - `get_gripper_state`
- 夹爪开度反馈 `actpos` 约为 0～1000；完全打开通常约 999/1000。
- `force` 是接口内部的力控等级/参数，不应直接解释为牛顿。
- 当前可乐瓶测试使用：`speed=100`、`force=200`，现场效果较稳定。
- 夹爪测试脚本：

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"
python test_native_gripper.py --speed 100 --force 200
```

该脚本只测试夹爪开合，不驱动机械臂移动。

## 4. 手眼标定结果与使用约定

### 4.1 眼在手上

左臂眼在手上使用：

```text
/home/lh/WRC/src/hand_eye_calibration/datasets/eye_in_hand_left/results/eye_in_hand_20_5_five_repeats.json
```

右臂使用对应的 `eye_in_hand_right` 结果文件。眼在手上时，使用手部 D435 和对应机械臂，标定板在外部固定，机械臂带着相机移动。

当前抓取测试模式示例：

```text
--mode eye_in_hand --arm left
```

### 4.2 眼在手外

眼在手外需要使用头部相机及头部 yaw/pitch 数据，或者使用相应动态眼在手外结果。左/右臂选择取决于目标所在区域。眼在手外的动态模型不是单个固定矩阵，而是按头部姿态建立的模型/插值或节点模型；使用时必须确认计算代码的模式和结果文件路径。

### 4.3 坐标与单位

- 网页 6D 位姿输入格式：`x, y, z, rx, ry, rz`
- 当前网页输出的平移通常为毫米，`run_pose_target.py` 会转换为米参与齐次矩阵计算。
- 旋转量为弧度的欧拉角，具体顺序必须保持现有代码约定，不要自行改成角度或更换旋转顺序。
- 目标链路一般为：

```text
camera_T_object
→ 手眼标定变换
→ base_T_object
→ 根据夹爪/TCP偏移计算法兰目标
→ 预抓取位姿与最终抓取位姿
```

- 当前夹爪中心相对法兰的 TCP 偏移已按法兰局部 `+Z` 方向处理：

```python
TCP_OFFSET_FLANGE_M = np.array([0.0, 0.0, 0.130])
```

不要随意改回旧的 `-Y` 偏移；此前已根据现场姿态和测试将方案改为局部 `+Z`。

## 5. 当前 `run_pose_target.py` 行为

当前代码已经实现以下完整阶段：

1. 读取命令行模式、机械臂和 6D 位姿。
2. 连接指定 Realman 机械臂。
3. 读取当前末端位姿和夹爪状态。
4. 读取对应眼在手上标定矩阵。
5. 计算 `base_T_object`、理论抓取法兰位姿、预抓取位姿和最终目标位姿。
6. 执行单位、矩阵、目标范围、逆解和关节跳变等安全检查。
7. 只有用户输入确认词后才发送机械臂运动命令。
8. 每次开始抓取前主动执行夹爪最大打开，并等待：

```text
actpos >= 990
```

9. 以 5% 速度执行 `MoveJ_P` 到预抓取位置。
10. 用户现场确认后，以 5% 速度执行非阻塞 `MoveL`，停在理论抓取点前 10 mm。
11. 用户再次确认后，输入 `FINAL GRASP LEFT`，以 2% 速度完成最后 10 mm 接近。
12. 以 `speed=100, force=200` 执行原生夹爪力控闭合。
13. 轮询机械臂与夹爪状态，超时、报错、停滞或夹爪异常时主动停止/重新打开。
14. 夹取成功后必须再次通过回退安全检查和确认词，按“基座+Z上升→沿接近轴反向退回→基座-Z下降→低速MoveJ_P回拍照位”执行；回到拍照位后保持夹爪闭合，不自动放置。

关键参数：

```python
FINAL_STOP_M = 0.010
MoveJ_P speed = 5%
pregrasp MoveL speed = 5%
final MoveL speed = 2%
MoveL timeout = 60 s
final approach timeout = 15 s
gripper speed = 100
gripper force = 200
return lift = base +Z, default 50 mm (allowed 30～50 mm)
return MoveL speed = 6%
return MoveL timeout = 90 s
return MoveJ_P speed = 6%
```

回退确认词：

```text
RETURN LEFT PHOTO
```

回退目标会在夹取成功后根据实际抓取末端位姿重新计算，并在任何回退运动前统一检查工作范围、移动距离、机械臂错误、逆解和单关节变化。检查失败时保持抓取位置，不发送回退命令。

## 6. 当前抓取启动命令

必须使用当前目录，不要进入旧的 `camera calibration` 目录：

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"
```

拍照位姿来源为统一文件 `/home/lh/WRC/src/control/grasp_pipeline/config/preset_poses.yaml`。当前只保留 `level_1/2/3_left/right` 六个货架拍照点；旧点位可从带时间戳备份恢复。旧 `initial_photo_pose.json` 只保留作历史核对。

采集左臂眼在手上样本：

```bash
python acquire_sample.py --mode eye_in_hand_left
```

采集左臂眼在手外样本：

```bash
python acquire_sample.py --mode eye_to_hand_left
```

采集移动速度可用 `--arm-speed` 和 `--lift-speed` 覆盖默认值，例如：

```bash
python acquire_sample.py --mode eye_in_hand_left --arm-speed 2 --lift-speed 5
```

左臂眼在手上抓取命令模板：

```bash
python run_pose_target.py \
  --mode eye_in_hand \
  --arm left \
  --standoff 0.20 \
  --return-lift-mm 50 \
  --pose X_MM Y_MM Z_MM RX RY RZ
```

例如最近一次测试的 6D 输入：

```bash
python run_pose_target.py \
  --mode eye_in_hand \
  --arm left \
  --standoff 0.20 \
  --pose \
  36.58656030893326 \
  -82.98677951097488 \
  580.9744596481323 \
  -0.6522907561320794 \
  -0.7407083343314502 \
  -2.773919768901571
```

运行时的交互口令：

```text
MOVE LEFT PREGRASP
APPROACH LEFT 10MM
FINAL GRASP LEFT
RETURN LEFT PHOTO
```

注意：口令必须在程序提示对应阶段后输入。不要在未检查空间安全、末端方向、夹爪两指位置和直线接近路径前确认。

## 7. 典型安全检查与故障处理

### 7.1 逆解失败或关节跳变过大

常见原因：

- 6D 位姿坐标系或单位错误。
- 目标物体姿态被错误解释为机械臂法兰姿态。
- TCP 偏移方向/长度不正确。
- 目标点离当前姿态过远。
- 目标位姿存在多个逆解，SDK选择了不连续的解。

程序出现 `[REJECTED]` 时不会发送运动命令。不要绕过安全检查，先读取当前位姿、检查矩阵和现场 TCP 方向。

### 7.2 MoveL 长时间不动

当前代码使用非阻塞 MoveL，并由 Python 轮询位姿，不依赖 SDK 的阻塞等待。若位置误差长期不下降，程序会在停滞/超时条件下发送 `move_stop()`。需要保留终端输出，确认误差是否持续下降。

### 7.3 夹爪未打开

每次运行都会重新发送最大打开命令并检查 `actpos`。如果 `actpos` 低于 990、`error != 0` 或状态异常，不应继续运动。可先运行 `test_native_gripper.py` 独立测试。

### 7.4 SSH 无图形界面

采集和抓取核心代码不依赖 OpenCV GUI；SSH 环境中 `DISPLAY` 为空时不要调用 `cv2.imshow`。可通过网页、本地查看已保存 RGB 图像或终端输出完成检查。

## 8. 已完成的标定采集与计算约定

### 8.1 眼在手外动态数据

眼在手外左臂数据曾按 200 组采集，每个头部节点约 20 组；计算逻辑使用随机 160 组训练、剩余数据验证，重复 5 次，并保存验证指标和最佳结果。曾支持 198 组有效样本，即 160 组训练、38 组验证。

### 8.2 眼在手上数据

眼在手上建议至少 15 组，推荐 20～25 组，重点是旋转变化、视野位置变化和距离变化，而不是单纯堆叠相似姿态。左臂眼在手上已完成数据采集并有 `eye_in_hand_20_5_five_repeats.json` 计算结果；右臂也有独立数据和结果目录。

## 9. 预设姿态与整体控制

整体预设姿态管理页面位于 `initial_pose_console` 相关目录，姿态统一保存为 YAML/JSON，包含：

- 躯干/升降柱实际高度（安全范围已确认约 700～1350 mm）。
- 头部 yaw、pitch。
- 左臂 6D 位姿。
- 右臂 6D 位姿。

一个预设点应包含完整字段；某个部件为 `null` 表示该预设点不移动该部件。例如左臂拍照点可以只填写躯干和左臂，头部和右臂为 `null`。控制代码读取预设点时只移动非 `null` 字段。

当前工作流的总体目标是：

```text
读取小票/货架信息
→ 回到对应预设拍照姿态
→ 采集 RGB/深度
→ 外部网页获得 camera_T_object
→ 选择 eye_in_hand 或 eye_to_hand
→ 坐标变换与安全检查
→ MoveJ_P 到预抓取
→ MoveL 直线接近
→ 夹爪闭合
→ 基座+Z上升
→ 沿接近轴反向退出
→ 基座-Z下降
→ MoveJ_P回拍照位
→ 后续实现放置
```

## 10. 后续开发原则

1. 不删除或覆盖已有标定数据集，尤其是 `datasets` 下的 RGB、pose 和结果文件。
2. 不恢复旧 `camera calibration` 路径；新增代码必须使用 `/home/lh/WRC/src`。
3. 修改运动代码后先做语法检查和只读解算，再做低速实机测试。
4. 任何新动作都先加入速度限制、超时、错误码检查、逆解检查和急停/停止路径。
5. 默认不自动跳过人工确认；正式抓取前必须保留预抓取检查点。
6. 网页只负责位姿估计或预设姿态管理；抓取执行由终端代码完成。
7. 所有新参数、坐标系约定、TCP 偏移和启动命令都同步更新本交接文档及相应目录 README。

## 11. 新 Codex 接手后的第一步

建议新会话先执行以下只读检查：

```bash
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"
python -m py_compile run_pose_target.py
rg -n "FINAL_STOP_M|TCP_OFFSET|gripper_release|FINAL GRASP|10MM" run_pose_target.py README.md
```

然后查看：

```bash
sed -n '1,260p' README.md
sed -n '1,220p' run_pose_target.py
```

未经现场确认，不要直接发送新的 MoveJ_P、MoveL 或夹爪闭合命令。
