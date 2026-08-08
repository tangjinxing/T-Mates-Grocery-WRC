# 小票识别服务

这个目录只负责一条运行链路：

```text
POST /receipt/parse
  -> GET 机器人头部相机当前帧
  -> Qwen3-VL 输出商品 name + specification
  -> SKU 服务校验商品名
  -> 返回标准 name + locations
```

调用方不上传图片。调用发生时相机看到的画面，就是本次识别的输入。

## 文件

```text
server.py                 本机小票服务（可选；远端感知也可用）
camera_snapshot_server.py 相机 HTTP 桥接（供远端 GET 拉图）
test_server.py            不访问真实服务的单元测试
README.md                 配置、启动与调用说明
```

## 相机 HTTP 桥接（给远端小票用）

远端 `POST http://192.168.130.59:8083/perception/parse` 会自己去拉相机图。  
在机器人上启动本桥接后，把远端的相机 URL 配成机器人局域网地址即可：

```bash
conda activate hand_eye_calib
cd /home/lh/WRC/src/parse_receipt
python camera_snapshot_server.py
# 默认 0.0.0.0:8083
```

本机验证：

```bash
curl -sS --noproxy '*' http://127.0.0.1:8083/health
curl -sS --noproxy '*' -o /tmp/head.jpg \
  'http://192.168.130.66:8083/camera/snapshot?camera=head&type=color'
```

远端应配置（示例，以机器人当前 WiFi IP 为准）：

```text
http://192.168.130.66:8083/camera/snapshot?camera=head&type=color
```

同一台 D435 不能被姿态台/手眼网页同时占用。

## 依赖

```bash
python -m pip install fastapi pillow uvicorn
```

正式接口不接收 multipart 文件，因此不需要 `python-multipart`。PaddleOCR 不在正式链路中。

## 配置

真实地址只通过环境变量配置，不写入仓库：

```bash
export RECEIPT_CAMERA_URL='http://<camera-host>:<port>/camera/snapshot?camera=head&type=color'
export QWEN_BASE_URL='http://<qwen-host>:<port>/v1'
export QWEN_MODEL='Qwen3-VL-4B-Instruct'
export QWEN_TIMEOUT_SECONDS='120'
export SKU_BASE_URL='http://127.0.0.1:25540'
export SKU_TIMEOUT_SECONDS='3'
export SKU_EDIT_DISTANCE_MAX='3'
```

如果 Qwen 需要认证，再设置：

```bash
export QWEN_API_KEY='<api-key>'
```

## 启动

在本目录运行：

```bash
uvicorn server:app --host 0.0.0.0 --port 8083
```

终端前台运行时，关闭终端或按 `Ctrl+C` 会停止服务。正式部署应由同事配置 systemd、Supervisor 或容器负责常驻和重启。

## 调用

服务器本机：

```bash
curl -X POST http://127.0.0.1:8083/receipt/parse
```

Mac 已将服务器 `8083` 转发到本地 `28083` 时：

```bash
curl -X POST http://127.0.0.1:28083/receipt/parse
```

请求体为空，不需要提供图片路径。机器人必须先到达小票拍摄位姿。

Qwen 内部输出只允许：

```json
[
  {
    "name": "康师傅香辣牛肉面",
    "specification": "500g"
  }
]
```

正式成功响应只保留 SKU 标准名称和位置：

```json
[
  {
    "name": "康师傅香辣牛肉面",
    "locations": ["H1_F_L1_C01"]
  }
]
```

Qwen 返回空数组时接口也返回 `[]`。商品名无法通过 SKU 精确匹配或编辑距离兜底时返回：

```json
{"error_code":"SKU_NOT_FOUND"}
```

## 一帧与三帧

当前 `/receipt/parse` 只 GET 一张相机图片。底层 `recognize_frames()` 已限制并支持 1–3 张同一小票图片；以后相机支持连续拍摄时，只需扩展采集函数，将三张图片一起传入，不需要重写 Qwen 和 SKU 链路。

## 测试

```bash
python -m unittest -v test_server.py
```

测试使用内存图片和模拟响应，不访问真实相机、Qwen 或 SKU，也不会保存图片文件。
