# IHMC Interfaces

Package that provides ROS2 interfaces for the IHMC communications layer.

### Dockerfile

The Dockerfile in the root of this project is used to create a Docker container based on Ubuntu 16.04 Xenial with ROS 2 Ardent
built from source and the IHMC `controller_msgs` package overlaid on top of it.

#### Building the docker image

The docker build requires Docker 17.05 or higher, as it uses a multi-stage build. It requires a build argument to establish which
ROS 2 release to build from source. The command for building a ROS 2 Ardent container:

```bash
$ docker build --build-arg ROS_VERSION_ARG=ardent -t ros2_ihmc_runtime:ardent .
```

And to run the container after building:

```bash
$ docker run -it ros2_ihmc_runtime:ardent /bin/bash 
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