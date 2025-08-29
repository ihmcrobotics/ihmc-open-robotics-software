# IHMC Perception Dependencies / Requirements

IHMC perception has some additional dependencies for using CUDA and running ZED cameras.
Follow the below instructions to install the dependencies.

Hardware requirements:
- A decent CPU
- NVIDIA 20-series or newer GPU

## Linux (Ubuntu 24.04, x86_64)
### CUDA
If there is any doubt, please follow the official installation instructions [here](https://developer.nvidia.com/cuda-12-9-1-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=24.04&target_type=deb_network
).

```
cd ~/Downloads
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-9
```

For modern GPUs (30-series and up)
```
sudo apt-get install -y nvidia-open
```

For older GPUs
```
sudo apt-get install -y cuda-drivers
```

### ZED SDK
We currently depend on ZED SDK 5.0.x, other versions are not compatible.

Download ZED SDK for Linux [here](https://www.stereolabs.com/developers/release/5.0#82af3640d775)


## Windows (10+ x86_64)
### CUDA
Download CUDA for Windows [here](https://developer.nvidia.com/cuda-12-9-1-download-archive?target_os=Windows&target_arch=x86_64)

### ZED SDK
Download ZED SDK for Windows [here](https://www.stereolabs.com/developers/release/5.0#82af3640d775)