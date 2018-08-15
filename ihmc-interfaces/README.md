# IHMC Interfaces

Package that provides ROS2 interfaces for the IHMC communications layer.

### Dockerfile

The Dockerfile in the root of this project is used to create a Docker container based on Ubuntu 16.04 Xenial with ROS 2 Ardent
built from source and the IHMC `controller_msgs` package overlaid on top of it.

#### Building the docker image

The docker build requires Docker 17.05 or higher, as it uses a multi-stage build. It requires a build argument to establish which
ROS 2 release to build from source. The command for building a ROS 2 Ardent container:

```bash
$ docker build -t ihmc_ros1_bridge:bouncy -f src/rosdocker/linux/Dockerfile .
```

#### Building to host, with log, on Windows

```bash
$ docker build -t ihmc_ros1_bridge:bouncy -f src/rosdocker/windows/Dockerfile .
$ docker run -v F:\dev\ros2bouncy:C:\dev\ros2 -it ihmc_ros1_bridge:bouncy

$ cd dev\ros2
$ curl -sk https://raw.githubusercontent.com/ros2/ros2/release-bouncy/ros2.repos -o ros2.repos
$ setx PATH "%PATH%;%ALLUSERSPROFILE%\chocolatey\bin;C:\OpenSSL-Win64\bin;C:\opencv\x64\vc15\bin;C:\Program Files\Git\cmd;C:\Program Files\CMake\bin;C:\Program Files\Cppcheck"
$ vcs import src < ros2.repos
$ call "N:\opensplice67\HDE\x86_64.win64\release.bat"
$ call "C:\Program Files\rti_connext_dds-5.3.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"
$ call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"
$ colcon build --merge-install --packages-skip rviz 2>&1 | tee log.txt
```

And to run the container after building:

```bash
$ docker run -it ros2_ihmc_runtime2:bouncy
```

### Trying out some examples

```bash
$ ros2 run ihmc_demo_py rcd_pub
```

```bash
$ ros2 run ihmc_demo_py rcd_sub
```

### Generating Messages
`gradle generateMessages`