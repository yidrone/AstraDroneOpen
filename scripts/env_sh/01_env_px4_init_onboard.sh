#!/bin/bash
# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ASTRA_DIR="$SCRIPT_DIR/../.."

echo "===== Starting MAVROS and QGC Setup ====="

# Install MAVROS
echo "Installing MAVROS and GeographicLib..."
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras

echo "Configuring GeographicLib..."
sudo cp -r "$ASTRA_DIR/third_party/GeographicLib/" /usr/share/
sudo cp "$ASTRA_DIR/AstraDrone_ros1_ws/src/Utils/devels/px4_config.yaml" /opt/ros/noetic/share/mavros/launch/

# QGroundControl Install (for ARM)
QGC_DIR="$HOME/QGC"
QGC_PATH="$QGC_DIR/QGroundControl.AppImage"
MIN_SIZE=50000000  # Set the minimum size to 50MB (50 * 1024 * 1024)
MAX_RETRIES=3
mkdir -p "$QGC_DIR"

ARCH_TYPE="$(uname -m)"
if [ "$ARCH_TYPE" = "aarch64" ]; then
    QGC_URL="https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.3/QGroundControl-4.4.3-aarch64.AppImage"
else
    echo "Unsupported architecture: $ARCH_TYPE"
    exit 1
fi

echo -e "\033[1;33mIf your download speed is too low, download manually:\033[0m"
echo -e "\033[1;34mhttps://github.com/mavlink/qgroundcontrol/releases/download/v4.4.3/QGroundControl-4.4.3-aarch64.AppImage\033[0m"
echo -e "\033[1;33mSave to: $QGC_PATH\033[0m"

# Function to check file size and retry download if necessary
download_qgc() {
    local attempt=1
    while [ $attempt -le $MAX_RETRIES ]; do
        # Download QGroundControl
        wget "$QGC_URL" -O "$QGC_PATH" && chmod +x "$QGC_PATH"
        
        # Check if file is large enough
        file_size=$(stat -c%s "$QGC_PATH")
        if [ "$file_size" -ge "$MIN_SIZE" ]; then
            echo -e "\033[1;32mQGroundControl downloaded successfully (size: $file_size bytes)\033[0m"
            return 0
        else
            echo -e "\033[1;31mFile size ($file_size bytes) is too small, deleting the file and retrying...\033[0m"
            rm -f "$QGC_PATH"
        fi
        
        attempt=$((attempt + 1))
        echo "Retrying... ($attempt/$MAX_RETRIES)"
        sleep 5
    done

    echo -e "\033[1;31mFailed to download QGroundControl after $MAX_RETRIES attempts.\033[0m"
    return 1
}

# # Start download and check
# if [ -f "$QGC_PATH" ]; then
#     file_size=$(stat -c%s "$QGC_PATH")
#     if [ "$file_size" -ge "$MIN_SIZE" ]; then
#         echo "QGroundControl exists and is valid. Setting permissions..."
#         sudo chmod +x "$QGC_PATH"
#     else
#         echo "File too small, re-downloading..."
#         if ! download_qgc; then
#             echo "Skipping QGroundControl installation due to download failure."
#             # Optionally, continue with the rest of the script
#         fi
#     fi
# else
#     echo "Downloading QGroundControl..."
#     if ! download_qgc; then
#         echo "Skipping QGroundControl installation due to download failure."
#         # Optionally, continue with the rest of the script
#     fi
# fi

# # Alias QGC
# QGC_ALIAS="alias qgc='$QGC_PATH'"
# if ! grep -qF "QGroundControl.AppImage" ~/.bashrc; then
#     echo "Adding QGC alias to .bashrc..."
#     echo -e "\n# QGroundControl alias" >> ~/.bashrc
#     echo "$QGC_ALIAS" >> ~/.bashrc
# else
#     echo "QGC alias already exists - skipping"
# fi

echo "===== Setup Complete ====="
echo -e "\033[1;33mYou can now use:\033[0m"
echo -e "\033[1;32mroslaunch mavros px4.launch\033[0m"
