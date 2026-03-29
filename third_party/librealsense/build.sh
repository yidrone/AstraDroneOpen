#!/bin/bash

echo "开始添加 Intel RealSense 仓库..."
# 添加 Intel RealSense 仓库并更新
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
echo "Intel RealSense 仓库添加完成."

echo "开始添加仓库公钥..."
# 添加仓库的公钥
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
echo "仓库公钥添加完成."

echo "开始更新并升级软件包..."
# 更新并升级软件包
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y
echo "软件包更新和升级完成."

echo "开始安装依赖包..."
# 安装依赖包
sudo apt-get install -y \
  libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
  git wget cmake build-essential \
  libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
  librscalibrationtool ros-noetic-ddynamic-reconfigure

echo "依赖包安装完成."

echo "开始应用补丁..."
# 应用补丁并设置 udev 规则
./scripts/patch-realsense-ubuntu-L4T.sh
echo "补丁应用完成."

echo "开始设置 udev 规则..."
# 设置 udev 规则
./scripts/setup_udev_rules.sh
echo "udev 规则设置完成."

echo "开始构建项目..."
# 构建项目
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true
make -j$(($(nproc)-1))
sudo make install
echo "项目构建和安装完成."


