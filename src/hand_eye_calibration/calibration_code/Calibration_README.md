# 手眼标定复现说明（简版）

> 更新日期：2026-08-05  
> 整理后项目根目录：`/home/lh/WRC/src/hand_eye_calibration`  
> 所有长度单位为米，机械臂姿态角为弧度；头部 yaw/pitch 是舵机原始读数，不是角度。

## 1. 运行环境

- Ubuntu 22.04，aarch64，Python 3.10
- Conda 环境：`hand_eye_calib`
- 主要依赖：NumPy 2.0.2、OpenCV Contrib 4.10、SciPy、PyYAML、
  pyrealsense2、FastAPI、Uvicorn

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
```

统一配置文件为 `calibration_config.yaml`。标定前必须确认配置与实机一致。

## 2. 硬件与接口对应关系

| 设备 | 地址/序列号 | Python 接口 |
|---|---|---|
| 左臂 | `169.254.128.18` | `/home/lh/robot_api/arm_api_new/realman_arm_api_api2.py`，`RealmanArmClient` |
| 右臂 | `169.254.128.19` | 同上 |
| 左手 D435 | `215322079194` | `/home/lh/robot_api/camera_api/D435_rgb_depth.py`，`D435Camera` |
| 右手 D435 | `335522072306` | 同上 |
| 头部 D435 | `344422070170` | 同上 |
| 头部 yaw/pitch | `/dev/ttyUSB0`，9600 baud | `/home/lh/robot_api/head_api1/servo_api.py`，`HeadControlSDK` |

头部舵机 ID：`1=pitch`，`2=yaw`。当前实机使用范围：

```text
yaw   350～650
pitch 400～500
到位允许误差：±8个舵机单位
```

同一台 D435 不能同时被两个进程打开。左右动态眼在手外页面可以同时启动，
但同一时刻只能在一个页面中连接头部相机。

## 3. 标定板

- 型号：ChArUco `CC300-20-15`
- 字典：`DICT_5X5_100`
- OpenCV 格点顺序：`14 × 9`
- 方格边长：`0.020 m`
- Marker 边长：`0.015 m`
- 最大 ChArUco 内角点：`(14-1) × (9-1) = 104`

标定不要求每张图都检测到 104 个角点；角点 ID 正确、覆盖范围充足、
姿态变化丰富更重要。标定板必须在整个采集过程中刚性固定，不能手持或松动。

网页检测采用当前单帧的原始灰度、CLAHE、Gamma 三路检测，选择角点数量、
覆盖率与 PnP 重投影质量较好的结果，再在原始灰度图上执行 `cornerSubPix`。
保存的 RGB 始终是原始 `1280×720 PNG`。

## 4. 四种标定模式

| 配置名 | 相机 | 机械臂 | 标定板放置 | 数据目录 | 输出含义 |
|---|---|---|---|---|---|
| `eye_in_hand_left` | 左手 D435 | 左臂 | 固定在外部 | `../datasets/eye_in_hand_left` | `left_gripper_T_camera` |
| `eye_in_hand_right` | 右手 D435 | 右臂 | 固定在外部 | `../datasets/eye_in_hand_right` | `right_gripper_T_camera` |
| `eye_to_hand_left` | 头部 D435 | 左臂 | 刚性固定在左臂末端 | `../datasets/dynamic_eye_to_hand_left` | `left_base_T_camera(yaw,pitch)` |
| `eye_to_hand_right` | 头部 D435 | 右臂 | 刚性固定在右臂末端 | `../datasets/dynamic_eye_to_hand_right` | `right_base_T_camera(yaw,pitch)` |

这里的机械臂位姿均为“所用机械臂基座到末端”的位姿，不是躯干统一世界坐标系。

## 5. 眼在手上代码与独立网页

眼在手上既可使用终端采集脚本，也可使用专用网页。专用网页不连接头部、不读取
yaw/pitch、不使用头部节点，并且不能与 8000/8001 动态眼在手外页面混用。

### 5.1 原有终端采集方式

```bash
# 左手相机 + 左臂：启动采集
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
python collect_calibration_data.py eye_in_hand_left

# 左臂采集结束后：计算 left_gripper_T_camera
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
python compute_calibration.py eye_in_hand_left

# 右手相机 + 右臂：启动采集
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
python collect_calibration_data.py eye_in_hand_right

# 右臂采集结束后：计算 right_gripper_T_camera
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
python compute_calibration.py eye_in_hand_right
```

采集命令：`Enter=保存`、`u=撤回上一组`、`q=退出`。
原始图、末端位姿、检测图和 `samples.json` 使用同一 sample ID 对应。

### 5.2 左右臂独立网页

左臂眼在手上（左臂 `169.254.128.18`、左手 D435 `215322079194`）：

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/web_interface
python start_left_eye_in_hand_web.py
```

访问：`http://169.254.128.40:8002/`

右臂眼在手上（右臂 `169.254.128.19`、右手 D435 `335522072306`）：

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/web_interface
python start_right_eye_in_hand_web.py
```

访问：`http://169.254.128.40:8003/`

两个页面的进程、相机序列号、机械臂地址和数据目录相互独立。连接硬件时，页面会从
对应 D435 读取当前彩色分辨率的工厂内参，并分别保存为：

```text
../datasets/eye_in_hand_left/camera_intrinsics_factory_1280x720.json
../datasets/eye_in_hand_right/camera_intrinsics_factory_1280x720.json
```

内参文件不能在左右相机之间互换，也不能沿用头部 D435 的内参。当前
`calibration.estimate_intrinsics: true` 表示最终计算还会使用同批 ChArUco 数据重新估计
内参；工厂内参用于实时 PnP 质量评估、留档和交叉核对。

网页提供实时 D435 图像、原图/CLAHE/Gamma 三路 ChArUco 检测、亚像素细化、亮度与
清晰度检查、当前机械臂末端位姿、姿态跨度提示、采集、拒绝、撤回和最终计算。

### 5.3 左臂眼在手上使用流程

1. 将 ChArUco 板刚性固定在机器人外部，采集期间标定板不能移动。
2. 确认左手 D435 序列号为 `215322079194`，左臂 IP 为 `169.254.128.18`。
3. 启动 8002 页面并连接硬件；每次人工移动左臂后，等待机械臂完全静止。
4. 确认标定板检测、亮度、清晰度和机械臂状态正常后点击“采集”，建议采集
   20～25 组，至少保证 15 组，
   并让相机相对标定板产生充分的位置变化和绕多个轴的旋转变化。
5. 数据完成后点击“结束并计算”；也可以退出网页后执行
   `python compute_calibration.py eye_in_hand_left` 重新计算。
6. 输出矩阵是 `left_gripper_T_camera`，即左臂末端坐标系到左手相机坐标系的固定变换；
   它不是头部相机到左臂基座的眼在手外矩阵。

右臂流程完全相同，但使用 8003 页面、右臂和右手 D435，输出为
`right_gripper_T_camera`。

页面数据和结果位置：

```text
../datasets/eye_in_hand_left/
  rgb/ poses/ detections/ rejected/ logs/ results/
  samples.json
  calibration_result.json
  results/calibration_result.json

../datasets/eye_in_hand_right/
  rgb/ poses/ detections/ rejected/ logs/ results/
  samples.json
  calibration_result.json
  results/calibration_result.json
```

当前数据目录为 `../datasets/eye_in_hand_left` 和
`../datasets/eye_in_hand_right`。后续完成眼在手上计算后，
应把实际结果文件路径、4×4 矩阵和验证误差补录到本文档，不能与第 8 节动态眼在手外结果混用。

## 6. 动态眼在手外独立页面

动态眼在手外每个样本同步保存：

1. 头部 D435 原始 RGB；
2. 头部实际 yaw/pitch；
3. 所选机械臂的 `base_T_gripper`；
4. ChArUco 检测质量与时间戳。

当前采用 10 个头部节点，每节点 20 组，共 200 组。每个节点内要人工移动机械臂，
保证末端位置和绕多个轴的旋转均有明显变化。

### 右臂页面

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/web_interface
python dynamic_handeye_web/web_calibration_app.py \
  --profile eye_to_hand_right --host 0.0.0.0 --port 8000
```

访问：`http://机器人主机IP:8000`

### 左臂页面

左侧入口固定使用左臂配置和独立数据目录：

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/web_interface
python start_left_eye_to_hand_web.py
```

访问：`http://机器人主机IP:8001`

左右页面共享检测/采集实现，但进程、端口、机械臂配置、数据目录和结果互相独立。
网页按钮包括连接、头部移动、采集、拒绝、撤回、上下节点与状态检查。

### 左臂动态眼在手外明确使用流程

1. 在机器人主机新开一个终端，逐行执行：

   ```bash
   conda activate hand_eye_calib
   cd /home/lh/WRC/src/hand_eye_calibration/web_interface
   python start_left_eye_to_hand_web.py
   ```

2. 保持该终端运行，在同一局域网电脑的浏览器访问
   `http://169.254.128.40:8001/`；若主机 IP 改变，替换为实际 IP。
3. 点击“连接硬件”，确认头部 D435、左臂、yaw/pitch 实际读数和实时图像均正常。
4. 依次移动到网页给出的 10 个头部节点。实际 yaw/pitch 必须进入目标值 ±8 的范围，
   计算时使用每条数据中保存的实际读数，而不是节点目标值。
5. 每个节点采集 20 组。每组都要先移动左臂末端并等待静止，确认角点、亮度、清晰度
   和机械臂状态合格后再点击“采集”；不合格画面使用“拒绝当前画面”。
6. 达到 200 组后，在运行网页的终端按 `Ctrl+C` 停止服务。网页的采集数据位于
   `../datasets/dynamic_eye_to_hand_left/`，不要手工修改或删除。
7. 新开终端执行计算：

   ```bash
   conda activate hand_eye_calib
   cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
   python compute_dynamic_160_40.py \
     --scene ../datasets/dynamic_eye_to_hand_left
   ```

8. 查看结果：
   `../datasets/dynamic_eye_to_hand_left/results/dynamic_160_40_five_repeats.json`。

## 7. 动态模型计算与验证

动态相机不是一个固定矩阵。计算模型为：

```text
base_T_camera(yaw,pitch)
  = T0 × Exp(xi_yaw × dyaw) × Exp(xi_pitch × dpitch)
```

独立计算脚本按每个节点随机抽取 16 组训练，剩余数据验证，随机重复 5 次，
以验证重投影误差选择最优模型：

```bash
# 右臂
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
python compute_dynamic_160_40.py \
  --scene ../datasets/dynamic_eye_to_hand_right

# 左臂
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
python compute_dynamic_160_40.py \
  --scene ../datasets/dynamic_eye_to_hand_left
```

若数据为完整 200 组，则是 `160训练/40验证`；若有效数据为 198 组，则是
`160训练/38验证`。脚本会只读排除撤回、重复索引和缺文件记录，不会改采集数据。
结果写入对应场景的 `results/`。

动态模型不能直接作为机械臂运动命令。使用时必须：

```text
读取头部实际yaw/pitch
→ 计算base_T_camera(yaw,pitch)
→ 将相机坐标点转换到对应机械臂基座
→ 再经过逆运动学、碰撞检测和低速安全运动
```

## 8. 当前项目状态与已知事项

- 右臂动态眼在手外：清单曾有 200 条有效记录，但存在重复索引 37、缺文件索引 71，
  实际独立有效数据为 198 组；计算使用 `160/38`，不要手工删除或改写历史记录。
- 右臂动态结果：
  `../datasets/dynamic_eye_to_hand_right/results/dynamic_160_38_five_repeats.json`。
- 左臂动态结果：
  `../datasets/dynamic_eye_to_hand_left/results/dynamic_160_40_five_repeats.json`。
- 头部命令值与实际值允许存在误差，计算必须使用每条 pose 文件中的实际读数。
- 机械臂和头部必须静止后采集；机械臂错误码非 0、角点不足、过暗或模糊时拒绝保存。

### 已完成的动态眼在手外计算结果

以下均为独立验证集上的 RMS。它们是移动头部条件下的动态
`arm_base_T_camera(yaw,pitch)` 模型结果，不是眼在手上的
`gripper_T_camera` 固定矩阵。

| 机械臂 | 有效样本与划分 | 最优重复 | 平移 RMS | 旋转 RMS | 重投影 RMS |
|---|---|---:|---:|---:|---:|
| 右臂 | 198 组，160 训练/38 验证 | 第 4 次 | `3.2262 mm` | `0.6437°` | `5.0536 px` |
| 左臂 | 200 组，160 训练/40 验证 | 第 5 次 | `3.0326 mm` | `0.7637°` | `5.5035 px` |

左臂第 5 次验证的完整摘要：平移平均值 `2.8715 mm`、RMS `3.0326 mm`、
最大值 `4.6936 mm`；旋转平均值 `0.7109°`、RMS `0.7637°`、最大值
`1.5232°`；重投影平均值 `5.1788 px`、RMS `5.5035 px`、最大值 `9.4122 px`。

右臂最优第 4 次验证摘要：平移 RMS `3.2262 mm`、旋转 RMS `0.6437°`、
重投影 RMS `5.0536 px`。模型参数保存在对应 JSON 的 `best_parameters`，运行时必须
结合实时 yaw/pitch 计算矩阵，不能把这组参数直接作为机械臂运动指令。

## 9. 数据保护原则

- 不直接删除历史样本；撤回操作将文件移动到 `rejected/`。
- 不手工修改 `samples.json`、pose JSON 或原始 RGB。
- 计算脚本只读采集数据，只向 `results/` 写新结果。
- 每次正式计算前检查：有效记录数、重复索引、缺失文件、各节点数量和图像分辨率。
