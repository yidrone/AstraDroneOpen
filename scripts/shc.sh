#!/bin/bash

# 检测当前系统架构
ARCH=$(uname -m)
echo "检测到系统架构: $ARCH"

# 根据架构设置输出文件名后缀
if [[ "$ARCH" == "x86_64" || "$ARCH" == "i386" || "$ARCH" == "i686" ]]; then
    SUFFIX="_x86.bin"
    echo "编译 x86 版本"
elif [[ "$ARCH" == "aarch64" || "$ARCH" == "armv7l" || "$ARCH" == "armv8l" ]]; then
    SUFFIX="_arm.bin"
    echo "编译 ARM 版本"
else
    SUFFIX=".bin"
    echo "未知架构，使用默认后缀"
fi

# 编译脚本
echo "开始编译..."

# 使用 [[ ]] 的条件判断
if [[ "$SUFFIX" == "_x86.bin" ]]; then
    shc -r -f pc_installer.sh -o "pc_installer${SUFFIX}"
    shc -r -f build_AstraDrone_ros1.sh -o "build_AstraDrone_ros1${SUFFIX}"
    shc -r -f build_sim_workspace.sh -o "build_sim_workspace${SUFFIX}"
else
    shc -r -f onboard_installer.sh -o "onboard_installer${SUFFIX}"
    shc -r -f build_AstraDrone_ros1.sh -o "build_AstraDrone_ros1${SUFFIX}"
fi


# 清理中间文件
rm -f *.x.c

echo "编译完成！生成的文件："
ls -la *${SUFFIX}