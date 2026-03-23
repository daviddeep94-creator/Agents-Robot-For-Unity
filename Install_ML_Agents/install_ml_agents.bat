@echo off
REM Unity ML-Agents 一键安装脚本
REM 适用于 Windows 系统

echo ========================================
echo Unity ML-Agents 一键安装脚本
echo ========================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未检测到 Python，请先安装 Python 3.8+
    echo 下载地址: https://www.python.org/downloads/
    pause
    exit /b 1
)

echo [1/5] 检查 Python 版本...
python --version
echo.

REM 升级 pip
echo [2/5] 升级 pip...
python -m pip install --upgrade pip --quiet
echo pip 已升级到最新版本
echo.

REM 卸载旧版本 (如果存在)
echo [3/5] 清理旧版本...
pip uninstall -y mlagents mlagents-envs 2>nul
echo 清理完成
echo.

REM 安装 ML-Agents
echo [4/5] 安装 ML-Agents...
pip install mlagents==1.1.0 --quiet
if %errorlevel% neq 0 (
    echo [错误] 安装失败，请检查网络连接
    pause
    exit /b 1
)
echo ML-Agents 安装成功
echo.

REM 验证安装
echo [5/5] 验证安装...
echo.
echo 已安装的包版本:
echo ------------------------------
pip show mlagents | findstr "Version"
pip show mlagents-envs | findstr "Version"
pip show torch | findstr "Version"
pip show tensorboard | findstr "Version"
echo ------------------------------
echo.

echo 安装完成！
echo.
echo 使用方法:
echo   1. 训练模型: mlagents-learn training_config.yaml --run-id=YourRunID
echo   2. 查看训练曲线: tensorboard --logdir=results
echo.
pause
