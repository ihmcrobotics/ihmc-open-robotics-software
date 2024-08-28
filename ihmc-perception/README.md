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

To install the appropriate NVIDIA driver:

```shell
sudo apt-get install -y nvidia-open
```

### nvCOMP

```shell
cd ~/Downloads
wget https://developer.download.nvidia.com/compute/nvcomp/3.0.5/local_installers/nvcomp_3.0.5_x86_64_12.x.tgz
sudo tar -xvf nvcomp_3.0.5_x86_64_12.x.tgz -C /usr/local/cuda/lib64/ --strip-components=1 lib/
sudo tar -xvf nvcomp_3.0.5_x86_64_12.x.tgz -C /usr/local/cuda/include/ --strip-components=1 include/
rm -f nvcomp_3.0.5_x86_64_12.x.tgz
```

### ZED SDK & API

```shell
cd ~/Downloads
wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.1/ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run
chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run
./ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run

# Follow the ZED SDK installer prompts

rm -f ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run
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

### nvCOMP

Some of the following commands must be ran as an administrator. To open a Command Prompt as administrator, search CMD in the windows search menu, right click on Command Prompt, and select "Run as administrator."

```shell
mkdir nvcomp
cd nvcomp
curl -o nvcomp.zip -L https://developer.download.nvidia.com/compute/nvcomp/3.0.5/local_installers/nvcomp_3.0.5_windows_12.x.zip
tar -xvf nvcomp.zip

:: The following move commands must be ran as administrator
move include\* "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v12.6\include"
move include\nvcomp "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v12.6\include"
move lib\nvcomp*.dll "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin"
move lib\nvcomp*.lib "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v12.6\lib\x64"

cd ..
rmdir /s /q nvcomp
```

### ZED SDK

```shell
cd %USERPROFILE%\Downloads
curl -o ZED_SDK_Installer.exe -L https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.1/ZED_SDK_Windows_cuda12.1_v4.1.3.exe
ZED_SDK_Installer .exe -s

:: Follow the ZED SDK installer prompts

del ZED_SDK_Installer.exe
```
