#!/bin/bash
# Unity ML-Agents 一键安装脚本
# 适用于 macOS/Linux 系统

echo "========================================"
echo "Unity ML-Agents 一键安装脚本"
echo "========================================"
echo ""

# 检查 Python 是否安装
if ! command -v python3 &> /dev/null; then
    echo "[错误] 未检测到 Python3，请先安装 Python 3.8+"
    echo "macOS: brew install python3"
    echo "Ubuntu: sudo apt-get install python3"
    exit 1
fi

echo "[1/5] 检查 Python 版本..."
python3 --version
echo ""

# 升级 pip
echo "[2/5] 升级 pip..."
python3 -m pip install --upgrade pip --quiet
echo "pip 已升级到最新版本"
echo ""

# 卸载旧版本 (如果存在)
echo "[3/5] 清理旧版本..."
pip3 uninstall -y mlagents mlagents-envs 2>/dev/null
echo "清理完成"
echo ""

# 安装 ML-Agents
echo "[4/5] 安装 ML-Agents..."
pip3 install mlagents==1.1.0 --quiet
if [ $? -ne 0 ]; then
    echo "[错误] 安装失败，请检查网络连接"
    exit 1
fi
echo "ML-Agents 安装成功"
echo ""

# 验证安装
echo "[5/5] 验证安装..."
echo ""
echo "已安装的包版本:"
echo "------------------------------"
pip3 show mlagents | grep "Version"
pip3 show mlagents-envs | grep "Version"
pip3 show torch | grep "Version"
pip3 show tensorboard | grep "Version"
echo "------------------------------"
echo ""

echo "安装完成！"
echo ""
echo "使用方法:"
echo "  1. 训练模型: mlagents-learn training_config.yaml --run-id=YourRunID"
echo "  2. 查看训练曲线: tensorboard --logdir=results"
echo ""
