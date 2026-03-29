#!/bin/bash

# 参数说明:
# $1 - 工作空间路径 (必填)
# $2 - 要处理的包集合路径 (必填)
# $3 - 操作类型: "add" 或 "remove" (必填)
# $4 - 文件名: ".catkin_ignore" 或 "CATKIN_IGNORE" (可选, 默认 CATKIN_IGNORE)

if [ $# -lt 3 ]; then
  echo "Usage: $0 <workspace_dir> <packages_root> <add|remove> [ignore_filename]"
  exit 1
fi

WORKSPACE_DIR="$1"
ROOT_DIR="$2"
ACTION="$3"
FILENAME="${4:-CATKIN_IGNORE}"  # 默认使用 CATKIN_IGNORE

echo "Processing packages in: $ROOT_DIR"
echo "Action: $ACTION $FILENAME"

find "$ROOT_DIR" -type f -name "package.xml" | while read pkg_file; do
    pkg_dir=$(dirname "$pkg_file")
    ignore_file="$pkg_dir/$FILENAME"
    
    case "$ACTION" in
        add)
            if [ -f "$ignore_file" ]; then
                echo "[SKIP] Already exists: $ignore_file"
            else
                touch "$ignore_file"
                echo "[ADD ] $ignore_file"
            fi
            ;;
        remove)
            if [ -f "$ignore_file" ]; then
                rm -f "$ignore_file"
                echo "[DEL ] $ignore_file"
            else
                echo "[SKIP] Not found: $ignore_file"
            fi
            ;;
        *)
            echo "Error: Invalid action '$ACTION'. Use 'add' or 'remove'"
            exit 1
            ;;
    esac
done

echo "✅ Operation completed for $(find "$ROOT_DIR" -name "package.xml" | wc -l) packages"
