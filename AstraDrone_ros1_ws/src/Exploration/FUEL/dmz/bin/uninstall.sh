#!/bin/bash
# 开启错误检测
set -e

CURRENT_USER=$(whoami)

# 防止用户使用 sudo 运行此脚本
if [ "$CURRENT_USER" == "root" ]; then
    echo "❌ 错误: 请不要使用 sudo 运行此脚本！直接运行 ./uninstall.sh 即可。"
    exit 1
fi

echo "================================================="
echo " 正在卸载 Ground API 系统服务..."
echo "================================================="

SERVICE_NAME="ground_api.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

# 1. 停止并禁用服务
echo "▶ 步骤 1: 停止并取消开机自启 (此时可能需要输入 sudo 密码)..."

# 检查服务是否在运行，如果在运行则停止
if systemctl is-active --quiet $SERVICE_NAME; then
    sudo systemctl stop $SERVICE_NAME
    echo "  - 已停止运行中的服务。"
fi

# 检查服务是否允许开机自启，如果是则禁用
if systemctl is-enabled --quiet $SERVICE_NAME 2>/dev/null; then
    sudo systemctl disable $SERVICE_NAME
    echo "  - 已取消开机自启。"
fi

# 2. 删除服务配置文件
echo "▶ 步骤 2: 删除系统服务配置文件..."
if [ -f "$SERVICE_PATH" ]; then
    sudo rm "$SERVICE_PATH"
    echo "  - 已删除文件 $SERVICE_PATH"
else
    echo "  - ⚠️ 提示: 配置文件不存在，可能之前已经卸载过了。"
fi

# 3. 重新加载 systemd 守护进程并清理残留状态
echo "▶ 步骤 3: 刷新系统状态..."
sudo systemctl daemon-reload
sudo systemctl reset-failed

echo "================================================="
echo "✅ 卸载完美结束！Ground API 服务已被彻底移除。"
echo ""
echo "注：本脚本仅卸载了系统服务，并未删除 Python 依赖库。"
echo "如果你想彻底清理依赖，可以手动执行："
echo "pip3 uninstall fastapi uvicorn pydantic pymavlink"
echo "================================================="