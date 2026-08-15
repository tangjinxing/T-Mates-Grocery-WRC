# Plan Pipeline

这是从 `../plan.py` 复制出的独立全流程版本。原文件保持不变。

## 文件

- `plan.py`：全流程编排。
- `config/service_endpoints.yaml`：按网络和运行主机保存服务地址。
- `requirements.txt`：额外 Python 依赖。

## 地址优先级

运行时地址按以下顺序决定：

1. `--perception-url` 等命令行参数；
2. `PERCEPTION_URL` 等环境变量；
3. YAML 中选中的 profile；
4. 旧代码默认地址。

## 在机器人上运行

```bash
cd /home/lh/WRC/src/plan_pipeline
python plan.py --profile robot_current_network --help
```

## 上传到 4090 后运行

```bash
python -m pip install -r requirements.txt
python plan.py --profile server_4090_current_network --help
```

需要切换整套网络地址时，新增或修改 YAML profile，不再修改 Python 源码。
临时覆盖单个接口时仍可使用环境变量，例如：

```bash
PERCEPTION_URL=http://新地址:8083 python plan.py --profile robot_current_network
```

程序启动时会打印最终生效的全部服务地址，执行真实动作前必须核对。
