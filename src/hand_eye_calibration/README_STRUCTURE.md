# 手眼标定整理目录说明

本目录由原项目
`/home/lh/WRC/camera calibration/hand_eye_calibration` 复制整理而来。
整理过程只复制内容，没有移动或删除原目录中的任何文件。

## 目录结构

```text
hand_eye_calibration/
├── calibration_code/  # 当前使用的数据采集、矩阵计算、配置和说明资料
├── web_interface/     # 眼在手外/眼在手上网页与启动入口
└── datasets/          # 四套标定数据、内参、检测图、日志和计算结果
```

## 数据目录

```text
datasets/
├── dynamic_eye_to_hand_left/
├── dynamic_eye_to_hand_right/
├── eye_in_hand_left/
└── eye_in_hand_right/
```

数据集是实际文件副本，不是指向旧目录的符号链接。整理完成时已使用
逐文件 checksum 核验新旧数据一致。

## 运行位置

计算脚本从 `calibration_code/` 执行：

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/calibration_code
```

网页入口从 `web_interface/` 执行：

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/web_interface
```

新副本中的 `calibration_config.yaml` 将数据根目录指向 `../datasets`。
`/home/lh/robot_api` 下的相机、机械臂和头部硬件接口仍通过绝对路径引用，未移动。

原 GitHub 仓库中未被当前方案使用的示例采集/计算脚本、辅助库、旧配置、旧 README
和文档图片没有保留在本整理目录中。原仓库暂时仍保留，可用于追溯。

## 数据保护

- 不要手工重命名 RGB、pose 或 detection 文件。
- 不要手工修改 `samples.json`。
- 原项目暂时保留；只有完成新目录的实机验证后才能考虑删除。
