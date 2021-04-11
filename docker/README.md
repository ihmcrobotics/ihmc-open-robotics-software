# IHMC Docker Images

Our images are hosted here: https://hub.docker.com/u/ihmcrobotics

To build an image:

```
docker/nvidia # docker build --tag ihmcrobotics/nvidia:0.1 .
```

To push an image:

```
# docker push ihmcrobotics/nvidia:0.1
```

To run an image to mess around in it's terminal:
```
# docker run --tty --interactive --net host ihmcrobotics/nvidia:0.1 bash
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

Some people want to run Docker without root. As far as we know, there isn't a secure way to do this and it's an exercise in frustation. However, in scripts, you can use `sudo -u root docker [...]` and `sudo -u robotlab docker [...]`, etc, to make sure docker is being run by a specific user. Ensure any scripts you make are being run as sudo by adding the following:
```
# Make sure it works one way or the other to reduce possible errors
if (( EUID == 0 )); then
    echo "Run with sudo." 1>&2
    exit 1
fi
```

To run these on Windows, install WSL 2 and follow the NVIDIA guide.
1. https://docs.microsoft.com/en-us/windows/wsl/install-win10
2. https://docs.nvidia.com/cuda/wsl-user-guide/index.html
