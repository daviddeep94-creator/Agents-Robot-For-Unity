# Unity ML-Agents 安装指南

## 版本信息

### Unity 端
| 包名 | 版本 | 说明 |
|------|------|------|
| com.unity.ml-agents | 2.0.2 | Unity ML-Agents 核心包 |
| com.unity.barracuda | 2.0.0 | 神经网络推理引擎 |
| com.unity.burst | 1.6.0+ | 高性能数学运算库 |

### Python 训练端
| 包名 | 版本 | 说明 |
|------|------|------|
| mlagents | 1.1.0 | ML-Agents 训练工具 |
| mlagents-envs | 1.1.0 | ML-Agents 环境 |
| torch | 2.1.1 | PyTorch 深度学习框架 |
| tensorboard | 2.20.0 | 训练可视化工具 |
| Python | 3.8-3.11 | Python 解释器 |

### 依赖包

#### 模型导出相关
| 包名 | 版本 | 说明 |
|------|------|------|
| onnx | 1.15.0 | ONNX 模型格式支持 (Unity 导出必需) |
| protobuf | 3.20.3 | Protocol Buffers 序列化 (ONNX 依赖) |
| h5py | 3.16.0 | HDF5 文件格式 (模型保存) |

#### 通信和环境
| 包名 | 版本 | 说明 |
|------|------|------|
| grpcio | 1.48.2 | Unity 与 Python 通信 (gRPC) |
| gym | 0.26.2 | 强化学习环境接口 |
| pettingzoo | - | 多智能体环境支持 |

#### 其他依赖
- numpy, pyyaml, pillow
- cloudpickle, filelock
- attrs, cattrs, six

---

## 一键安装

### Windows 系统

双击运行 `install_ml_agents.bat` 文件，或在命令行执行:

```powershell
.\install_ml_agents.bat
```

### macOS/Linux 系统

```bash
chmod +x install_ml_agents.sh
./install_ml_agents.sh
```

---

## 手动安装步骤

### 1. 安装 Python (3.8+)

下载地址: https://www.python.org/downloads/

安装时勾选 **"Add Python to PATH"**

验证安装:
```bash
python --version
```

### 2. 升级 pip

```bash
python -m pip install --upgrade pip
```

### 3. 安装 ML-Agents

```bash
pip install mlagents==1.1.0 -i https://pypi.tuna.tsinghua.edu.cn/simple

# 修复版本冲突（必须按此顺序执行）
pip install --force-reinstall numpy==1.23.5 onnx==1.13.0 protobuf==3.20.3 -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install onnxscript -i https://pypi.tuna.tsinghua.edu.cn/simple

pip install torch==1.13.1 torchvision==0.14.1 --index-url https://download.pytorch.org/whl/cpu
pip install onnxruntime==1.14.1
```

**版本说明**：
- mlagents 1.1.0：官方要求 onnx==1.15.0，但为兼容 onnxscript 使用 1.17.0
- numpy 1.23.5：mlagents 要求 <1.24.0,>=1.23.5
- protobuf 3.20.3：mlagents 要求 <3.21,>=3.6
- onnxscript：模型导出必需，要求 onnx>=1.17

### 4. 验证安装

```bash
# 检查安装版本
pip show mlagents
pip show mlagents-envs
pip show torch
pip show tensorboard

# 测试训练命令
mlagents-learn --help
```

---

## 训练模型

### 基本命令

```bash
# 训练颠球模型
mlagents-learn "F:\GameProject\AI Robot\Assets\ML-Agents\training_config.yaml" `
  --run-id=BallJuggleBehavior `
  --results-dir="F:\GameProject\AI Robot\Assets\ML-Agents\MLModels" `
  --force

# 训练机器人平衡模型
mlagents-learn "F:\GameProject\AI Robot\Assets\ML-Agents\training_config.yaml" `
  --run-id=RobotBalanceBehavior `
  --force

# 训练机器人移动模型
mlagents-learn "F:\GameProject\AI Robot\Assets\ML-Agents\training_config.yaml" `
  --run-id=RobotMovementBehavior `
  --force
```

### 常用参数

| 参数 | 说明 |
|------|------|
| `--run-id` | 训练任务名称 (必需) |
| `--results-dir` | 结果保存目录 |
| `--force` | 覆盖同名训练 |
| `--resume` | 继续之前的训练 |
| `--initialize-from` | 从已有的训练加载 |
| `--num-envs` | 并行环境数量 |

---

## 查看训练曲线

```bash
# 启动 TensorBoard
tensorboard --logdir="F:\GameProject\AI Robot\Assets\ML-Agents\MLModels"

# 或在结果目录下
tensorboard --logdir=results

# 在浏览器打开 http://localhost:6006
```

---

## 训练配置文件

配置文件位置: `Assets/ML-Agents/training_config.yaml`

### BallJuggleBehavior 配置
```yaml
max_steps: 10000000
batch_size: 512
buffer_size: 5120
learning_rate: 3.0e-4
hidden_units: 256
num_layers: 3
```

### RobotBalanceBehavior 配置
```yaml
max_steps: 100000
batch_size: 2048
buffer_size: 20480
learning_rate: 3.0e-4
hidden_units: 256
num_layers: 2
```

### RobotMovementBehavior 配置
```yaml
max_steps: 1000000
batch_size: 2048
buffer_size: 20480
learning_rate: 3.0e-4
hidden_units: 512
num_layers: 3
```

---

## 导出和使用模型

### 模型导出流程

ML-Agents 训练完成后会自动将模型导出为 ONNX 格式:

1. **自动导出时机**
   - 训练过程中定期生成检查点 (`.nn` 格式)
   - 训练完成后自动导出最终模型为 `.onnx` 格式

2. **手动导出模型**
   ```bash
   # 使用 mlagents-export 工具
   mlagents-export <path-to-checkpoint.nn> --output-dir <output-directory>

   # 示例
   mlagents-export "results/BallJuggleBehavior/BallJuggleBehavior-1000000.nn" `
     --output-dir "Assets/ML-Agents/Models"
   ```

3. **模型导出依赖**
   - **onnx** (1.15.0): ONNX 格式支持
   - **protobuf** (3.20.3): 模型序列化
   - **torch**: PyTorch 模型转换
   - 这些依赖已包含在 `pip install mlagents` 中

### 1. 训练完成后模型位置
- 目录: `Assets/ML-Agents/MLModels/{BehaviorName}/`
- 文件: `{BehaviorName}.onnx`

### 2. 在 Unity 中使用模型
1. 将 `.onnx` 文件导入 Unity
2. 选择 Agent 对象
3. 在 `BehaviorParameters` 组件的 `Model` 字段中指定模型
4. 设置 `Behavior Type` 为 `Inference Only`
5. 运行场景测试

### 3. 模型导出格式说明

| 格式 | 扩展名 | 用途 |
|------|--------|------|
| 检查点 | `.nn` | PyTorch 格式，用于继续训练 |
| 导出模型 | `.onnx` | ONNX 格式，用于 Unity 推理 |

**注意**: Unity Barracuda 引擎只支持 ONNX 格式，训练完成后必须导出为 `.onnx` 文件才能在 Unity 中使用。

---

## 常见问题

### Q: 安装失败，提示网络错误?
A: 使用国内镜像源安装:
```bash
pip install mlagents==1.1.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Q: 训练时提示 "Previous data from this run ID was found"?
A: 使用 `--force` 参数覆盖旧数据:
```bash
mlagents-learn config.yaml --run-id=YourID --force
```

### Q: 如何指定使用 GPU 训练?
A: 确保安装了 CUDA 版本的 PyTorch:
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### Q: 训练速度太慢怎么办?
A: 增加并行环境数量:
```bash
mlagents-learn config.yaml --num-envs=10
```

### Q: 如何查看完整的训练参数?
A: 运行帮助命令:
```bash
mlagents-learn --help
```

### Q: 如何手动导出模型为 ONNX 格式?
A: 使用 mlagents-export 工具:
```bash
mlagents-export <checkpoint.nn> --output-dir <output-directory>
```

### Q: 模型导出失败怎么办?
A: 检查导出相关依赖:
```bash
# 验证 ONNX 相关包
pip show onnx protobuf

# 重新安装缺失的包
pip install onnx protobuf --upgrade
```

---

## 版本兼容性

| Unity 版本 | ML-Agents 包 | Python 版本 | mlagents 版本 |
|------------|-------------|-------------|---------------|
| 2019.4+    | 2.0.2       | 3.8-3.11    | 1.1.0         |

注意: Unity 端 2.0.2 包需要 Python 端 1.1.0 训练工具配合使用。

---

## 卸载

```bash
pip uninstall -y mlagents mlagents-envs
```

---

## 参考资料

- Unity ML-Agents 官方文档: https://github.com/Unity-Technologies/ml-agents
- ML-Agents API 文档: https://github.com/Unity-Technologies/ml-agents/blob/main/docs/ML-Agents-Toolkit.md
