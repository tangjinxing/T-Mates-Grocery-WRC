# 动态头部眼在手外网页标定

启动后，用同一局域网电脑访问服务端显示的网址。网页负责头部节点控制、D435 实时预览、ChArUco 检测、机械臂只读状态、同步采集、撤回和标定计算。

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/hand_eye_calibration/web_interface
python dynamic_handeye_web/web_calibration_app.py --profile eye_to_hand_right --host 0.0.0.0 --port 8000
```

浏览器访问 `http://169.254.128.40:8000`。首次使用前必须检查 `calibration_config.yaml` 中的 `head_port`、安全范围和 10 个头部节点。

左臂动态眼在手外使用独立入口和端口：

```bash
python start_left_eye_to_hand_web.py
```

浏览器访问 `http://169.254.128.40:8001`，数据只写入
`../datasets/dynamic_eye_to_hand_left`。左右网页共享实现，但配置、进程、端口、数据和结果相互独立。
同一台头部D435不能被两个页面同时连接。

结果保存于 `../datasets/dynamic_eye_to_hand_right/results/`。动态计算报告包含模型参数和验证指标；运行时必须结合实际 yaw/pitch 计算 `base_T_camera(yaw,pitch)`。
