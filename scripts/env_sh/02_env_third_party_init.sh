#!/bin/bash
# 功能：
# 1. 自动编译第三方库
# 2. 自动记录 make install 安装到系统的文件
# 3. 下次运行根据记录文件检查库是否存在，存在则跳过，不存在则重新编译

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
THIRD_PARTY_DIR="$SCRIPT_DIR/../../third_party"

#-------------------------------
# 工具函数：检查记录文件并判断是否完整安装
#-------------------------------
check_installed() {
    local record_file="$1"

    if [ ! -f "$record_file" ]; then
        return 1
    fi

    echo "检测到安装记录: $record_file"
    while IFS= read -r line; do
        if [ ! -f "$line" ]; then
            echo "缺失文件: $line"
            return 1
        fi
    done < "$record_file"

    return 0
}

#-------------------------------
# 自动编译与安装函数
#-------------------------------
compile_install() {
    local pkg_name="$1"
    local source_dir="$THIRD_PARTY_DIR/$pkg_name"
    local build_dir="$source_dir/build"
    local record_file="$source_dir/install_files.txt"

    echo "=================================================="
    echo "处理组件: $pkg_name"
    echo "源码目录: $source_dir"
    echo "构建目录: $build_dir"
    echo "=================================================="

    # 1. 若已安装，则跳过
    if check_installed "$record_file"; then
        echo "✔ $pkg_name 已安装且文件完整 → 跳过编译"
        return
    fi

    echo "❌ 未找到完整安装 → 开始编译安装 $pkg_name"

    # 2. 清理构建目录
    rm -rf "$build_dir"
    mkdir -p "$build_dir"

    cd "$build_dir" || exit 1

    # 3. CMake 配置
    if [ -f "$source_dir/CMakeLists.txt" ]; then
        cmake "$source_dir" || { echo "CMake 配置失败: $pkg_name"; exit 1; }
    else
        echo "❌ 错误：未找到 CMakeLists.txt！路径：$source_dir"
        exit 1
    fi

    # 4. 编译
    CORES=$(nproc)
    make -j$((CORES + 1)) || { echo "编译失败: $pkg_name"; exit 1; }

    # 5. 捕获 make install 将安装哪些文件
    echo "===== 记录 make install 文件 ====="
    install_tmp="/tmp/${pkg_name}_install_log"
    rm -rf "$install_tmp"
    mkdir -p "$install_tmp"

    make install DESTDIR="$install_tmp"

    # 写入记录文件
    > "$record_file"
    find "$install_tmp" -type f | while read f; do
        real="${f#$install_tmp}"
        real="/$real"
        echo "$real" >> "$record_file"
    done

    echo "安装文件清单写入: $record_file"

    # 6. 真正安装到系统
    sudo make install || { echo "安装失败: $pkg_name"; exit 1; }

    echo "✔ 成功安装: $pkg_name"
    echo
}

#-------------------------------
# 主执行函数
#-------------------------------
main() {
    # 自动检测 CPU 架构（arm / x86）
    ARCH=$(uname -m)
    echo "检测到系统架构: $ARCH"

    if [[ "$ARCH" == "aarch64" || "$ARCH" == arm* ]]; then
        echo "当前为 ARM 架构，使用 ARM 优化配置"
        IS_ARM=true
    else
        echo "当前为 x86 架构"
        IS_ARM=false
    fi


    echo "开始安装第三方依赖..."

    compile_install "Livox-SDK" "."
    compile_install "Livox-SDK2" "."
    compile_install "nlopt" "."
    compile_install "apriltag" "."
}

main

