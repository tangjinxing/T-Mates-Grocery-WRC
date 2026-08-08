# 整体预设姿态管理台

该页面用于控制和查看头部、躯干、左右臂，并按预设名称分别将实际状态保存到 `/home/lh/WRC/src/control/grasp_pipeline/config/preset_poses.yaml`。四个部件不需要同时连接，每次保存只更新当前选择预设的一个字段，并自动备份旧YAML。页面仍只管理当前选中的一台D435。

## 启动

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/control/initial_pose_console
python app.py
```

局域网访问：`http://169.254.128.40:8010/`

如果主机当前使用其他局域网地址，请将 URL 中的 IP 替换成 `hostname -I` 显示的对应地址，端口仍为 `8010`。

## 统一预设保存

页面可以选择 `robot_initial_safe`、`receipt_scan` 和已有货架拍照预设，也可以按命名规则创建新预设。分别使用：

```text
保存实际头部 yaw/pitch
保存实际躯干高度
保存实际左臂6D
保存实际右臂6D
```

每个预设还必须勾选 `apply_components`（躯干、头部、左臂、右臂）并保存执行范围。后续控制代码只移动勾选部件；参与执行的部件没有实际数据时，预设状态为 `pending/incomplete` 并禁止执行；未勾选部件即使为 `null` 也保持不动。

机械臂保存各自基座系下的实际末端6D位姿 `[x,y,z,rx,ry,rz]`，位置单位为米、姿态单位为XYZ欧拉角弧度。躯干高度读取和运动依赖配置中的左臂控制器连接。升降柱目标高度限制为100～1350 mm、速度限制为1～20%，运动需要浏览器二次确认。

## 旧版独立JSON

- 左臂：`config/poses/left_initial_photo_pose.json`
- 右臂：`config/poses/right_initial_photo_pose.json`
- 头部：`config/poses/head_initial_photo_pose.json`

旧JSON暂时保留用于历史核对；页面主流程不再使用它们保存新预设。

头部没有 6D 笛卡尔位姿接口，因此保存实际 `yaw/pitch` 舵机读数，并在能读取到时同时记录升降柱高度。覆盖已有预设前会生成 `.bak.json` 备份。

## 安全约束

- 页面启动后不会自动连接、启动相机或移动机器人。
- 任意运动都需要二次确认；左右臂运动速度限制为 `1～10%`。
- 左右臂共用运动互斥锁，不允许网页同时下发两条运动任务。
- 升降柱已确认使用100～1350 mm范围，当前 `control_config.yaml` 中 `lift.enable_motion: true`，页面允许在该范围内调节目标高度。
- “紧急停止运动”会向已连接的左右臂和升降柱发送停止命令，但不能代替实体急停。

## 当前预设与常见问题

- 当前统一预设文件是 `/home/lh/WRC/src/control/grasp_pipeline/config/preset_poses.yaml`。
- 当前已验收的左臂眼在手上拍照预设为 `shelf_upper_photo_eye_in_hand_left`；左臂眼在手外拍照预设为 `shelf_upper_photo_eye_to_hand_left`。
- 页面保存机械臂位姿时使用各自机械臂基座系，位置单位米、姿态单位 XYZ 欧拉角弧度；头部保存 yaw/pitch 舵机原始读数。
- 页面无响应时先检查8010端口旧进程；连接硬件失败时确认对应相机/机械臂没有被其他网页或终端程序占用。
- 访问地址中的 IP 不是固定世界地址，主机换网后用 `hostname -I` 获取当前局域网 IP，端口仍为8010。

## 建议验收顺序

1. 先切换三台相机，确认任一时刻只有一台工作，画面比例为 16:9。
2. 分别连接左右臂，只检查实际 6D 位姿与关节角读数。
3. 连接头部，只检查实际 yaw/pitch 与升降柱状态。
4. 清空运动范围中的人员与障碍物后，再用当前实际值进行一次低速原位运动测试。
5. 调整到拍照姿态后保存 JSON，再点击“查看 JSON”核对坐标系与单位。
