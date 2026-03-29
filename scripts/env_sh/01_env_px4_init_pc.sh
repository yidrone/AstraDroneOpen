#!/bin/bash
# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ASTRA_DIR="$SCRIPT_DIR/../.."

echo "===== Starting PX4 Environment Setup ====="
echo -e "\033[1;33mNOTE: PX4 source download may require scientific internet access!\033[0m"

PX4_DIR="$HOME/PX4-Autopilot"

# Clone PX4
if [ ! -d "$PX4_DIR" ]; then
    echo "Downloading PX4 source code (v1.15.4)..."
    git clone -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" || {
        echo -e "\033[1;31mError: Failed to clone PX4 repository\033[0m"
        exit 1
    }
else
    echo "PX4 directory already exists at $PX4_DIR - skipping clone"
fi

cd "$PX4_DIR" || { echo "Error: PX4 directory not found"; exit 1; }

echo "Installing pyulog 1.1.0..."
pip install pyulog==1.1.0

# Clean old build
echo "Cleaning previous PX4 builds..."
make clean
rm -rf build

# Install PX4 dependencies
echo "Running PX4 setup script..."
./Tools/setup/ubuntu.sh -y

# Launch PX4 build and Gazebo in new terminal
echo "Launching PX4 build and Gazebo in new terminal..."
gnome-terminal -- bash -c "
    echo '=== PX4 Build + Gazebo Start ===';
    cd '$PX4_DIR';
    CMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
    make px4_sitl gazebo;
    echo '=== PX4 Build/Gazebo process exited ===';
    exec bash
"

# Wait for gzserver to start (max 100min)
echo "PX4源码在另一个窗口编译中, 请耐心等待编译, 等待gazebo打开之后即可手动关闭新的终端, 本终端会继续后续内容! "
echo "Waiting for Gazebo server (gzserver) to start..."
MAX_WAIT=6000
WAIT_TIME=0
while ! pgrep -x gzserver > /dev/null; do
    sleep 10
    WAIT_TIME=$((WAIT_TIME + 5))
    if [ "$WAIT_TIME" -ge "$MAX_WAIT" ]; then
        echo -e "\033[1;31mError: Timeout waiting for gzserver to start.\033[0m"
        echo -e "\033[1;33mPlease check if PX4 + Gazebo started successfully in new terminal.\033[0m"
        exit 1
    fi
done
echo -e "\033[1;32mGazebo detected running (gzserver). Continuing setup...\033[0m"

# Update .bashrc
echo "Checking environment variables configuration..."
CONFIG_MARKER="# ===== PX4 Auto-Config (Added by 01_px4_init.sh) ====="
if ! grep -qF "$CONFIG_MARKER" ~/.bashrc; then
    echo "Adding PX4 config to .bashrc..."
    cat << EOF >> ~/.bashrc

$CONFIG_MARKER
source $PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash \\
    $PX4_DIR \\
    $PX4_DIR/build/px4_sitl_default
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_DIR
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic
# ===== End of PX4 Config =====
EOF
else
    echo "PX4 config already exists in .bashrc - skipping"
fi

# Install MAVROS
echo "Installing MAVROS and GeographicLib..."
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras

echo "Configuring GeographicLib..."
sudo cp -r "$ASTRA_DIR/third_party/GeographicLib/" /usr/share/
sudo cp "$ASTRA_DIR/AstraDrone_ros1_ws/src/Utils/devels/px4_config.yaml" /opt/ros/noetic/share/mavros/launch/

# Apply env
echo "Applying environment settings..."
source ~/.bashrc

# QGroundControl Install
QGC_DIR="$HOME/QGC"
QGC_PATH="$QGC_DIR/QGroundControl.AppImage"
MIN_SIZE=170000000
mkdir -p "$QGC_DIR"

ARCH_TYPE="$(uname -m)"
if [ "$ARCH_TYPE" = "x86_64" ] || [ "$ARCH_TYPE" = "aarch64" ]; then
    QGC_URL="https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.3/QGroundControl.AppImage"
else
    echo "Unsupported architecture: $ARCH_TYPE"
    exit 1
fi

echo -e "\033[1;33mIf your download speed is too low, download manually:\033[0m"
echo -e "\033[1;34mhttps://github.com/mavlink/qgroundcontrol/releases/download/v4.4.3/QGroundControl.AppImage\033[0m"
echo -e "\033[1;33mSave to: $QGC_PATH\033[0m"

if [ -f "$QGC_PATH" ]; then
    file_size=$(stat -c%s "$QGC_PATH")
    if [ "$file_size" -gt "$MIN_SIZE" ]; then
        echo "QGroundControl exists and valid. Setting permissions..."
        sudo chmod +x "$QGC_PATH"
    else
        echo "File too small, re-downloading..."
        wget "$QGC_URL" -O "$QGC_PATH" && chmod +x "$QGC_PATH"
    fi
else
    echo "Downloading QGroundControl..."
    wget "$QGC_URL" -O "$QGC_PATH" && chmod +x "$QGC_PATH"
fi

# Alias QGC
QGC_ALIAS="alias qgc='$QGC_PATH'"
if ! grep -qF "QGroundControl.AppImage" ~/.bashrc; then
    echo "Adding QGC alias to .bashrc..."
    echo -e "\n# QGroundControl alias" >> ~/.bashrc
    echo "$QGC_ALIAS" >> ~/.bashrc
else
    echo "QGC alias already exists - skipping"
fi

echo "===== Setup Complete ====="
echo -e "\033[1;32mPX4 installed at: $PX4_DIR\033[0m"
echo -e "\033[1;32mQGC installed at: $QGC_PATH\033[0m"
echo -e "\033[1;33mYou can now use:\033[0m"
echo -e "\033[1;32mroslaunch px4 mavros_posix_sitl.launch\033[0m"