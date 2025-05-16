#!/bin/bash

echo "Downloading models..."
sudo chown -R $USER:$USER FoundationPose/
cd FoundationPose || exit 1
mkdir -p weights

# Install gdown if not available
if ! command -v gdown &> /dev/null; then
    echo "Installing gdown..."
    pip3 install --break-system-packages gdown
fi

# Function to check presence of config.yml and model_best.pth
check_weights_exist() {
    local dir=$1
    [[ -f "$dir/config.yml" && -f "$dir/model_best.pth" ]]
}

# Pose estimation weights
EST_DIR="weights/2024-01-11-20-02-45"
EST_ZIP="$EST_DIR.zip"
if check_weights_exist "$EST_DIR"; then
    echo "[✓] Pose estimation weights already present. Skipping."
elif [ -f "$EST_ZIP" ]; then
    echo "[>] Found zip for pose estimation. Extracting..."
    unzip -o "$EST_ZIP" -d weights/
    rm -f "$EST_ZIP"
else
    echo "[↓] Downloading pose estimation weights..."
    gdown 1cyI3wKcdWAWyXZZrsVmhcLLk9qfL_fRi -O "$EST_ZIP"
    unzip -o "$EST_ZIP" -d weights/
    rm -f "$EST_ZIP"
fi

# Pose refinement weights
REF_DIR="weights/2023-10-28-18-33-37"
REF_ZIP="$REF_DIR.zip"
if check_weights_exist "$REF_DIR"; then
    echo "[✓] Pose refinement weights already present. Skipping."
elif [ -f "$REF_ZIP" ]; then
    echo "[>] Found zip for pose refinement. Extracting..."
    unzip -o "$REF_ZIP" -d weights/
    rm -f "$REF_ZIP"
else
    echo "[↓] Downloading pose refinement weights..."
    gdown 15gBTLShNNPRoYJwRwkOB_neeWZy-34Vb -O "$REF_ZIP"
    unzip -o "$REF_ZIP" -d weights/
    rm -f "$REF_ZIP"
fi


