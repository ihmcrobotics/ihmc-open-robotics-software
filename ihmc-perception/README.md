# IHMC Perception Dependencies

IHMC perception has some additional dependencies for using CUDA and running ZED cameras.
Follow the below instructions to install the dependencies.

## Linux (Ubuntu 22.04, x86_64)

### CUDA Toolkit

For other linux distros or architectures, find installation instructions on the [CUDA Toolkit Download](https://developer.nvidia.com/cuda-downloads) site.

```shell
cd ~/Downloads
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-6
```

To install the NVIDIA driver for 20 series graphics cards and up:

```shell
sudo apt-get install -y nvidia-open
```

For older NVIDIA graphics cards: 

```shell
sudo apt-get install -y cuda-drivers
```

### ZED SDK

```shell
cd ~/Downloads
wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Ubuntu22_cuda12.1_v4.2.1.zstd.run
chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.2.1.zstd.run
./ZED_SDK_Ubuntu22_cuda12.1_v4.2.1.zstd.run

# Follow the ZED SDK installer prompts

rm -f ZED_SDK_Ubuntu22_cuda12.1_v4.2.1.zstd.run
```

## Windows (x86_64)

### CUDA Toolkit

```shell
cd %USERPROFILE%\Downloads
curl -OL https://developer.download.nvidia.com/compute/cuda/12.6.0/network_installers/cuda_12.6.0_windows_network.exe
cuda_12.6.0_windows_network.exe -s

:: Follow the NVIDIA Installer prompts. 
:: When asked whether to run express or custom installation, you may select express. 

del cuda_12.6.0_windows_network.exe
```

### ZED SDK

```shell
cd %USERPROFILE%\Downloads
curl -o ZED_SDK_Installer.exe -L https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Windows_cuda12.1_v4.2.1.exe
ZED_SDK_Installer .exe -s

:: Follow the ZED SDK installer prompts

del ZED_SDK_Installer.exe
```
