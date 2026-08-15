# WRC Capability API

当前版本提供真实姿态准备动作。调用前必须确认机器人周围和双臂路径安全。

启动：

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/capability_api
python app.py
```

启动时会按照`config/hardware_init.yaml`依次初始化并打开左右原生夹爪。启动服务前必须确认两个夹爪均未持物。初始化失败不会关闭姿态准备能力，但对应手臂的真实抓取会被拒绝。

接口：

- `GET /health`
- `GET /pose/health`
- `POST /pose/prepare`
- `GET /manipulation/health`
- `POST /manipulation/grasp`（`execute=false`只读规划；`execute=true`真实抓取并回退）
- `POST /manipulation/release`（`execute=false`只读放置链检查；`execute=true`真实放置并返回READY）

`POST /pose/prepare`必须携带`Idempotency-Key`：

```json
{"pose_id":"level_1"}
```

成功返回`{"status":"SUCCEEDED"}`；失败返回非2xx和`EXECUTION_FAILED`。

物理动作幂等状态持久化在`runtime/idempotency.sqlite3`。相同键在服务重启后仍会重放原终态；遗留`RUNNING`状态禁止自动重试，需要人工核查机器人状态。

姿态动作开始前会一次性连接并检查所有参与硬件，校验躯干和头部范围，并完成左右臂目标逆解及单关节变化检查；全部通过后才发送第一条运动命令。

抓取只读规划请求示例：

```bash
curl --noproxy '*' -X POST http://机器人IP:8099/manipulation/grasp \
  -H 'Content-Type: application/json' \
  -H 'Idempotency-Key: grasp-plan-001' \
  -d '{
    "pose":[19.99,-101.03,620.03,-2.57,-0.26,-0.19],
    "hand":"RIGHT",
    "frame":"camera",
    "pose_unit":"mm_rad",
    "rotation_order":"zyx"
  }'
```

`rotation_order`会先按输入顺序构造旋转矩阵，再规范化为现有抓取算法使用的XYZ欧拉角；不会直接交换三个角。`execute=false`仅执行手眼变换、候选姿态、全过程逆解及安全筛选；`execute=true`会在重新规划通过后自动完成抓取和安全回退。抓取图像采集后到调用本接口前，目标手臂不得移动，因为当前实际末端位姿被视为拍照瞬间位姿。

`GET /manipulation/health`会返回左右夹爪的启动初始化结果。只有目标手臂`initialized=true`时，接口才接受`execute=true`；只读规划不依赖夹爪初始化状态。

放置只读检查请求示例：

```bash
curl --noproxy '*' -X POST http://机器人IP:8099/manipulation/release \
  -H 'Content-Type: application/json' \
  -H 'Idempotency-Key: release-left-plan-001' \
  -d '{"hand":"LEFT","execute":false}'
```

真实放置将`execute`改为`true`并使用一个全新的`Idempotency-Key`。接口读取
`capability_poses.yaml`中的`DELIVERY_TABLE_PLACE_READY`以及目标手臂对应的
`DELIVERY_TABLE_PLACE_TRANSITION_LEFT/RIGHT`和
`DELIVERY_TABLE_PLACE_FINAL_LEFT/RIGHT`。真实执行前要求躯干处于400±10mm、
目标手臂位于READY、夹爪处于`mode=6`力控夹持状态，并完成
`READY -> TRANSITION -> FINAL -> TRANSITION -> READY`全过程逆解、单关节变化和
J5奇异性检查。执行顺序为到过渡点、到最终点、松爪、返回过渡点、返回READY。
