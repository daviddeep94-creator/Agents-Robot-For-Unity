# 颠球训练项目 (Ball Juggle Training)

这是一个简单的Unity ML-Agents颠球训练项目，训练AI控制板子让球保持在板子上方。

## 项目简介

这个项目实现了ML-Agents的基础使用，包括：
- 简单的物理模拟（球体和板子）
- ML-Agents Agent实现
- 观测空间和动作空间设计
- 奖励函数设计
- 训练配置

## 文件结构

```
Assets/
├── Scripts/
│   ├── BallJuggleAgent.cs      # 主要的Agent脚本
│   └── BallTrigger.cs          # 球落地检测脚本
├── Editor/
│   └── BallJuggleSetup.cs      # 编辑器辅助工具
├── ML-Agents/
│   └── training_config.yaml    # 训练配置（包含BallJuggleBehavior）
└── Scenes/
    └── BallJuggle.unity       # 颠球训练场景
```

## 快速开始

### 1. 设置场景

在Unity编辑器中，可以通过以下方式快速设置场景：

**方法1：使用编辑器工具**
```
菜单: Tools > Ball Juggle > Setup Training Scene
```

**方法2：使用编辑器窗口**
```
菜单: Tools > Ball Juggle > Show Setup Window
```

在窗口中点击"创建完整场景"按钮。

### 2. 手动设置场景

如果手动设置，需要创建以下对象：

1. **板子 (Paddle)**
   - 创建一个Cube
   - 位置: (0, 0, 0)
   - 缩放: (2, 0.1, 2)
   - 添加BoxCollider

2. **球体 (Ball)**
   - 创建一个Sphere
   - 位置: (0, 2, 0)
   - 缩放: (0.3, 0.3, 0.3)
   - Tag设置为"Ball"
   - 添加Rigidbody（Mass=0.1, Drag=0.01）
   - 设置物理材质（Bounciness=0.8）

3. **Agent对象**
   - 创建空GameObject命名为"BallJuggleAgent"
   - 添加`BallJuggleAgent`脚本
   - 在Inspector中配置参数：
     - **Ball**: 拖拽球体对象到此字段
     - **Paddle**: 拖拽板子对象到此字段
   - 添加`BehaviorParameters`组件（ML-Agents核心组件）：
     - **Behavior Name**: `BallJuggleBehavior`（与training_config.yaml中的名称对应）
     - **Vector Observation**:
       - **Space Size**: `8`（观测维度）
       - **Stacked Vectors**: `1`（堆叠向量数，默认1）
     - **Actions**:
       - **Continuous Actions**: `2`（2维连续动作：X和Z方向移动）
       - **Discrete Branches**: `0`（离散动作分支数，连续动作设为0）
     - **Model**:
       - **Inference Device**: `CPU`（推理设备，可选CPU或GPU）
     - **Behavior Type**: `Default`（默认行为，支持训练和推理）
     - **Team Id**: `0`（团队ID，默认0）
     - **Use Child Sensors**: `False`（是否使用子对象的传感器）
     - **Observable Attribute Handling**: `Ignore`（可观察属性处理方式）
   - 添加`DecisionRequester`组件：
     - **Decision Period**: `5`（每5帧做一次决策）

4. **地面触发器 (GroundTrigger)**
   - 创建一个Plane
   - 位置: (0, -5, 0)
   - 禁用MeshRenderer
   - 设置BoxCollider为Trigger
   - 添加`BallTrigger`脚本，指向Agent

5. **相机设置**
   - 位置: (0, 3, -8)
   - 旋转: (15, 0, 0)

### 3. 训练配置

训练配置已添加到`training_config.yaml`中的`BallJuggleBehavior`部分。

关键参数：
- `max_steps`: 500000（最大训练步数）
- `batch_size`: 1024
- `buffer_size`: 10240
- `learning_rate`: 3.0e-4
- `time_horizon`: 64
- `hidden_units`: 128
- `num_layers`: 2

### 4. 开始训练

在命令行中运行：

```bash
# 方法1：使用双引号包裹完整路径
mlagents-learn "F:\GameProject\AI Robot\Assets\ML-Agents\training_config.yaml" `
--run-id=BallJuggleBehavior `
--results-dir="F:\GameProject\AI Robot\Assets\ML-Agents\MLModels" `
--force
```

**注意**：
- 如果遇到 "Previous data from this run ID was found" 错误，说明该run-id已被使用过，可以：
  - 使用新的run-id：`--run-id=BallJuggleTraining2`
  - 或添加 `--force` 参数覆盖旧数据：`mlagents-learn ... --run-id=BallJuggleTraining --force`
  - 或添加 `--resume` 参数继续之前的训练：`mlagents-learn ... --run-id=BallJuggleTraining --resume`
- 如果遇到 `encoding_size` 废弃警告，可以忽略，不影响训练

### 5. 测试训练结果

训练完成后，可以将训练好的模型应用到Agent：
1. 训练完成后会在 `ML-Agents/results/BallJuggleTraining/` 目录下生成 `.onnx` 模型文件
2. 在Unity中找到 BallJuggleAgent 对象的 **BehaviorParameters** 组件
3. 将生成的 `.onnx` 文件拖拽到 **Model** 字段中
4. 确认 **Behavior Type** 设置为 `Inference Only`（仅推理模式）
5. 运行场景，AI将自动控制板子保持球的平衡

## Agent设计说明

### 观测空间 (8维)

1. 球相对于板子的位置 (x, y, z) - 归一化
2. 球的速度 (vx, vy, vz) - 归一化
3. 板子相对于初始位置的偏移 (x, z) - 归一化

### 动作空间

**Continuous Actions: 2**（连续动作）
1. 动作[0]: 板子水平X方向移动
2. 动作[1]: 板子水平Z方向移动

**Discrete Branches: 0**（离散动作分支数，本项目不使用离散动作）

### 奖励函数

- **基础奖励**: 每帧+0.1（球在板子上方时）
- **位置奖励**: 球越靠近板子中心，奖励越高
- **偏移惩罚**: 球偏离中心时给予小幅惩罚
- **失败惩罚**: 球落地时-1并结束Episode

### 终止条件

- 球落到y < -0.5时结束Episode
- 其他条件可根据需要添加

## 手动测试

在Heuristic模式下，可以使用键盘控制板子：
- **W/S**: 前后移动（对应Z轴）
- **A/D**: 左右移动（对应X轴）

在Unity中设置BallJuggleAgent的 **BehaviorParameters** 组件的 **Behavior Type** 为 `Heuristic Only` 即可使用键盘控制。

## 训练建议

1. **初期训练**: 设置较小的max_steps（如10000）快速验证
2. **完整训练**: 使用默认的500000步数获得更好的性能
3. **超参数调整**:
   - 如果学习太慢，增大learning_rate
   - 如果不稳定，减小learning_rate或增大buffer_size
   - 如果探索不够，增大gamma

## 常见问题

**Q: 训练时球不落地？**
A: 检查物理材质的Bounciness设置，确保设置为0.8左右

**Q: Agent不移动板子？**
A: 检查DecisionRequester的Decision Period设置，建议设为5

**Q: 训练没有进展？**
A: 
- 检查观测空间是否正确配置
- 检查奖励函数是否合理
- 尝试调整学习率或网络结构

**Q: 如何增加难度？**
A:
- 减小球体尺寸
- 增加板子移动范围
- 增加球体初速度的随机性

## 扩展思路

可以在此基础上进行扩展：
1. 添加多个球体
2. 添加障碍物
3. 板子可以旋转
4. 目标移动
5. 多Agent协作

## 参考资料

- Unity ML-Agents文档: https://github.com/Unity-Technologies/ml-agents
- 原视频教程: https://www.bilibili.com/video/BV1hE411W7Pi
- 项目源码: https://github.com/3D-Wizard/3D-Wizard-World

## 许可证

本项目用于学习目的，请遵守相关开源协议。
