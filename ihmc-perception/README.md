# IHMC Perception Dependencies

IHMC perception has some additional dependencies for using CUDA and running ZED cameras.
Follow the below instructions to install the dependencies.

## Linux (Ubuntu 22.04, x86_64)

### CUDA Toolkit

For other linux distros or architectures, find installation instructions on the [CUDA Toolkit Download](https://developer.nvidia.com/cuda-downloads) site.

```shell
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-6
```

To install the appropriate Nvidia driver:

```shell
sudo apt-get install -y nvidia-open
```

### nvCOMP

```shell
wget https://developer.download.nvidia.com/compute/nvcomp/3.0.5/local_installers/nvcomp_3.0.5_x86_64_12.x.tgz
sudo tar -xvf nvcomp_3.0.5_x86_64_12.x.tgz -C /usr/local/cuda/lib64/ --strip-components=1 lib/
sudo tar -xvf nvcomp_3.0.5_x86_64_12.x.tgz -C /usr/local/cuda/include/ --strip-components=1 include/
rm -f nvcomp_3.0.5_x86_64_12.x.tgz
```

### ZED SDK & API

Install the SDK:

```shell
wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.1/ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run
chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run
./ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run

# Follow the ZED SDK installer prompts

rm -f ZED_SDK_Ubuntu22_cuda12.1_v4.1.3.zstd.run
```

Install the API:

```shell
# Download and unzip API
wget -O zed-c-api.tar.gz https://codeload.github.com/stereolabs/zed-c-api/tar.gz/refs/tags/v4.1.0
tar -xvf zed-c-api.tar.gz

# Build and install the API
cd zed-c-api-4.1.0/
mkdir build
cd build
cmake ..
make
sudo make install

# Clean up
cd ../..
rm -f zed-c-api.tar.gz
rm -rf zed-c-api-4.1.0/
```

## Windows (x86_64)

### CUDA Toolkit

```shell
curl -OL https://developer.download.nvidia.com/compute/cuda/12.6.0/network_installers/cuda_12.6.0_windows_network.exe
cuda_12.6.0_windows_network.exe -s

:: Follow the NVIDIA Installer prompts. 
:: When asked whether to run express or custom installation, you may select express. 

del cuda_12.6.0_windows_network.exe
```

### nvCOMP

```shell
curl -o nvcomp.zip https://developer.download.nvidia.com/compute/nvcomp/4.0.0/local_installers/nvcomp-windows-x86_64-4.0.0-cuda12.5.zip
tar -xvf nvcomp.zip

:: The following move commands must be ran as administrator
move nvcomp\include\* "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v12.6\include"
move nvcomp\lib\nvcomp*.dll "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin"
move nvcomp\lib\nvcomp*.lib "%ProgramFiles%\NVIDIA GPU Computing Toolkit\CUDA\v12.6\lib\x64"

del nvcomp.zip
rmdir /s /q nvcomp
```

