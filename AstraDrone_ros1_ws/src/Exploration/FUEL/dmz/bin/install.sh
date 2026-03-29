#!/bin/bash
# 开启错误检测
set -e

CURRENT_USER=$(whoami)

# 防止用户使用 sudo 运行此脚本
if [ "$CURRENT_USER" == "root" ]; then
    echo "❌ 错误: 请不要使用 sudo 运行此脚本！直接运行 ./install.sh 即可。"
    exit 1
fi

echo "================================================="
echo " 正在自动化部署 Ground API 环境与系统服务..."
echo "================================================="

# 1. 安装 Python 核心依赖
echo "▶ 步骤 1: 正在安装 Python 依赖 (FastAPI, Uvicorn, Pymavlink)..."
pip3 install --user fastapi uvicorn pydantic pymavlink open3d numpy

# 2. 自动获取当前脚本所在的绝对路径！(黑科技在这里)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
UVICORN_BIN="/home/$CURRENT_USER/.local/bin/uvicorn"
SERVICE_PATH="/etc/systemd/system/ground_api.service"

echo "📍 检测到当前工作目录为: $SCRIPT_DIR"

# 检查当前目录下有没有 ground_api.py
if [ ! -f "$SCRIPT_DIR/ground_api.py" ]; then
    echo "❌ 致命错误: 未在当前目录 ($SCRIPT_DIR) 找到 ground_api.py！"
    echo "请确保你把此脚本放在了 ground_api.py 同级目录下。"
    exit 1
fi

# 3. 生成 systemd 服务文件
echo "▶ 步骤 2: 正在创建 systemd 服务 (此时可能会提示输入 sudo 密码)..."

# 使用动态获取的 $SCRIPT_DIR 写入服务配置
sudo bash -c "cat > $SERVICE_PATH" <<EOF
[Unit]
Description=AstraDrone Ground Control API
After=network.target

[Service]
User=$CURRENT_USER
Environment=USER=$CURRENT_USER
Environment=HOME=/home/$CURRENT_USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$UVICORN_BIN ground_api:app --host 0.0.0.0 --port 8080 --no-access-log
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# 4. 重新加载并启动服务
echo "▶ 步骤 3: 正在重新加载守护进程并设置开机自启..."
sudo systemctl daemon-reload
sudo systemctl enable ground_api.service
sudo systemctl restart ground_api.service

echo "================================================="
echo "✅ 部署完美结束！Ground API 已在后台启动，并已配置开机自启。"
echo "================================================="