# IHMC Docker Images

Our images are hosted here: https://hub.docker.com/u/ihmcrobotics

We recommend running Docker commands without `sudo`. To enable this,
follow the [official instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user):
```
# groupadd docker
# usermod -aG docker $USER
$ newgrp docker
```

To build an image:

```
docker/nvidia $ docker build --tag ihmcrobotics/nvidia:0.1 .
```

To push an image:

If you haven't yet or in a while, login to Docker:
```
$ docker login
```

Then push the image:

```
$ docker push ihmcrobotics/nvidia:0.1
```

To run an image to mess around in it's terminal:
```
$ docker run --tty --interactive --net host ihmcrobotics/nvidia:0.1 bash
```

To add GPU acceleration, add the following and test with `clinfo`.
```
--privileged
--gpus all
--device /dev/dri:/dev/dri
```

If you want to display a GUI, add the following and test with `glxinfo` and `glxgears`.
To show a GUI on an X server, you must first allow it with `xhost +local:docker`.
```
--env DISPLAY
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw
```

Sometimes volume mount permissions can be wrong. To debug this, use `ls -hanp` to list files with the group and owner codes as numbers. You should use the same codes for the user inside the Docker container as the user on the host system. Usually, this is 1000.

To run these on Windows, install WSL 2 and follow the NVIDIA guide.
1. https://docs.microsoft.com/en-us/windows/wsl/install-win10
2. https://docs.nvidia.com/cuda/wsl-user-guide/index.html

OpenCL doesn't work on WSL yet. https://github.com/microsoft/WSL/issues/6951
