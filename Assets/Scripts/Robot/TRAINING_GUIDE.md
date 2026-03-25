# AI机器人训练指南

本文档介绍如何训练人形机器人AI，包括两个阶段：站立平衡和目标移动。

## 目录
1. [环境准备](#环境准备)
2. [训练阶段1：站立平衡](#训练阶段1站立平衡)
3. [训练阶段2：目标移动](#训练阶段2目标移动)
4. [训练参数调整](#训练参数调整)
5. [常见问题](#常见问题)

---

## 环境准备

### 1. 安装Unity ML-Agents

```bash
pip install mlagents
pip install torch
pip install tensorboard
```

### 2. 创建训练场景

1. 打开Unity编辑器
2. 菜单：Window > AI Robot > Articulation Robot Generator
3. 生成一个机器人
4. 菜单：Window > AI Robot > Training Scene Setup
5. 按照步骤添加组件：
   - 添加训练管理器
   - 添加平衡Agent
   - 添加移动Agent
   - 创建训练地面

### 3. 配置Agent行为名称

在Unity中为两个Agent设置不同的Behavior Name：
- **平衡Agent**: `RobotBalanceBehavior`
- **移动Agent**: `RobotMovementBehavior`

---

## 训练阶段1：站立平衡

### 目标
让机器人学会在平面上保持直立姿态，不倾倒。

### 奖励机制
- **正向奖励**：
  - 保持直立（每帧 +0.01）
  - 接近目标高度（最多 +10）
  - 向目标高度移动（+变化量 × 10）
  - 达到目标站立时间（+1）

- **负向惩罚**：
  - 倾斜角度过大（最多 -0.1）
  - 关节过度移动（-移动量 × 0.001）
  - 失败（-1）

### 观测空间（35维）
1. 机器人高度（归一化）
2. 机器人旋转（3维欧拉角，归一化）
3. 垂直速度（归一化）
4. 角速度（3维，归一化）
5. 12个关节角度（归一化）
6. 12个关节速度（归一化）
7. 重力方向（局部坐标）

### 动作空间
- 12个连续动作，对应12个主要关节
- 每个动作范围：[-1, 1]

### 训练配置

```yaml
max_steps: 500000
time_horizon: 1000
learning_rate: 3.0e-4
hidden_units: 256
num_layers: 2
gamma: 0.995
```

### 训练命令

```bash
# 方法1：使用双引号包裹完整路径
mlagents-learn "F:\GameProject\AI Robot\Assets\ML-Agents\training_config.yaml" `
--run-id=RobotBalanceAgent `
--results-dir="F:\GameProject\AI Robot\MLModels" 

mlagents-learn "E:\Agents-Robot-For-Unity\Assets\ML-Agents\training_config.yaml" `
--run-id=RobotBalanceAgent `
--results-dir="E:\Agents-Robot-For-Unity\MLModels"

# 方法2：使用相对路径
C:\Users\Administrator\AppData\Local\Programs\Python\Python310\python.exe -m mlagents-learn "Assets\ML-Agents\training_config.yaml" `
--run-id=RobotBalanceAgent `
--results-dir="Assets\ML-Agents\MLModels" 

# 方法3：使用打包后的exe训练
mlagents-learn "E:\Agents-Robot-For-Unity\Assets\ML-Agents\training_config.yaml" `
--run-id=RobotBalanceAgent `
--env="E:\Agents-Robot-For-Unity\Build\AI Robot.exe" `
--results-dir="E:\Agents-Robot-For-Unity\Assets\ML-Agents\MLModels" `
--time-scale=20 `
--num-envs=15 `
--resume 
# 训练完成后查看结果
tensorboard --logdir=results
```

### 成功标准
- 平均奖励 > 50
- 能够稳定站立 > 10秒
- 倾斜角度 < 10度

---

## 训练阶段2：目标移动

### 目标
在站立平衡的基础上，让机器人向目标点移动。

### 前提条件
- 完成阶段1平衡训练
- 加载阶段1的模型检查点

### 奖励机制
- **正向奖励**：
  - 接近目标（+0.01 × 距离变化量）
  - 保持直立（+0.005）
  - 到达目标（+2）

- **负向惩罚**：
  - 倾斜角度过大（最多 -0.1）
  - 速度过快（-速度 × 0.01）
  - 关节过度移动（-移动量 × 0.0005）
  - 失败（-1）

### 观测空间（40维）
在平衡阶段基础上增加：
1. 相对目标位置（3维归一化）
2. 目标距离（1维归一化）

### 动作空间
- 18个连续动作（增加更多关节控制）
- 每个动作范围：[-1, 1]

### 训练配置

```yaml
max_steps: 1000000
time_horizon: 1000
learning_rate: 3.0e-4
hidden_units: 512
num_layers: 3
gamma: 0.99
```

### 训练命令

```bash
# 从平衡训练的检查点继续训练
mlagents-learn training_config.yaml --run-id=MovementTraining --initialize-from=BalanceTraining

# 或从头开始训练
mlagents-learn training_config.yaml --run-id=MovementTraining
```

### 成功标准
- 平均奖励 > 100
- 能够可靠到达目标（成功率 > 80%）
- 移动过程中保持平衡

---

## 训练参数调整

### 关键参数说明

#### RobotBalanceAgent
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `targetStandTime` | 10秒 | 成功站立的目标时间 |
| `targetHeight` | 0.875m | 目标站立高度 |
| `maxTiltAngle` | 30度 | 最大允许倾斜角度 |
| `uprightReward` | 0.01 | 每帧保持直立奖励 |
| `jointForceScale` | 100 | 关节力矩缩放 |

#### RobotMovementAgent
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `targetSpawnRange` | 5m | 目标生成范围 |
| `minTargetDistance` | 1m | 最小目标距离 |
| `maxTargetDistance` | 3m | 最大目标距离 |
| `approachReward` | 0.01 | 接近目标奖励系数 |
| `jointForceScale` | 150 | 关节力矩缩放 |

### 常见调整策略

#### 1. 机器人总是摔倒
- 降低 `jointForceScale`（减少力矩）
- 增加 `tiltPenaltyMultiplier`（增加倾斜惩罚）
- 降低目标高度 `targetHeight`

#### 2. 训练收敛太慢
- 增加 `learning_rate`
- 增加网络容量（`hidden_units`, `num_layers`）
- 调整奖励函数，增加正向奖励

#### 3. 机器人过度摆动
- 增加 `jointMovementPenalty`（关节移动惩罚）
- 减小 `jointForceScale`
- 添加速度限制

#### 4. 无法到达目标
- 降低目标距离（`maxTargetDistance`）
- 增加 `approachReward`（接近奖励）
- 延长Episode时间（`maxEpisodeTime`）

---

## 常见问题

### Q1: 如何监控训练进度？
A: 使用TensorBoard查看训练曲线：
```bash
tensorboard --logdir=results
```

### Q2: 如何保存和加载模型？
A: 训练会自动保存检查点到 `results/` 目录。加载模型：
```bash
mlagents-learn config.yaml --run-id=NewTraining --initialize-from=OldTraining
```

### Q3: 如何使用训练好的模型？
A: 在Unity中：
1. 将训练好的 `.onnx` 文件导入到 `Assets/ML-Agents/Models/`
2. 在Agent组件的 `Model` 属性中指定模型文件
3. 设置 `Behavior Type` 为 `Inference`

### Q4: 训练需要多长时间？
A:
- 平衡训练：约1-3小时（取决于硬件）
- 移动训练：约3-6小时（取决于硬件）

### Q5: 可以使用多个机器人并行训练吗？
A: 可以！在场景中放置多个机器人（每个都有Agent），可以显著加速训练。

### Q6: 为什么机器人只是原地不动？
A:
- 检查奖励函数是否合理
- 确认关节力矩足够大（`jointForceScale`）
- 增加探索参数（epsilon, beta）

### Q7: 如何测试训练结果？
A:
1. 将Agent的 `Behavior Type` 设置为 `Inference`
2. 加载训练好的模型
3. 在Unity中运行场景
4. 可以按 `B` 键启用 `Heuristic` 模式手动测试

---

## 进阶技巧

### 1. Curriculum Learning
- 从简单任务开始，逐渐增加难度
- 例如：先在平面上训练，再在斜面上训练

### 2. Reward Shaping
- 使用中间奖励指导学习
- 避免稀疏奖励导致学习困难

### 3. Domain Randomization
- 随机化物理参数（质量、摩擦等）
- 提高模型泛化能力

### 4. Multi-Agent Training
- 同时训练多个机器人
- 可以使用竞争或协作机制

### 5. Transfer Learning
- 使用预训练模型加速学习
- 例如：用平衡模型初始化移动训练

---

## 脚本说明

### RobotBalanceAgent.cs
- 功能：平衡训练Agent
- 关键方法：
  - `CollectObservations()`: 收集状态信息
  - `OnActionReceived()`: 执行动作
  - `CalculateAndApplyReward()`: 计算奖励

### RobotMovementAgent.cs
- 功能：移动训练Agent
- 继承自平衡训练的概念，增加目标导航

### TrainingManager.cs
- 功能：训练流程管理
- 支持阶段切换、统计显示、数据保存

### TrainingSceneSetup.cs
- 功能：快速设置训练场景
- Editor窗口，一键添加所需组件

---

## 许可证

MIT License

## 联系方式

如有问题或建议，请提交Issue或Pull Request。
