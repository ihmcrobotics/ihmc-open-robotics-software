# IHMC Interfaces

Package that provides ROS2 interfaces for the IHMC communications layer.

### Dockerfile

The Dockerfile in the root of this project is used to create a Docker container based on Ubuntu 16.04 Xenial with ROS 2 Ardent
built from source and the IHMC `controller_msgs` package overlaid on top of it.

The Linux version of this Docker image can also be used to create a portable package of the `ros1_bridge` with our messages built in to it. See the instructions
at the bottom of the README: ***"Building the ROS 1 Bridge"***

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

### Building the ROS 1 Bridge

Building the ROS 1 Bridge starts with using the Linux Dockerfile, as building the bridge also requires a working ROS 1 install which the Linux dockerfile sets up.

A `docker build` will create an image that can be started which has a valid bridge inside of it; these instructions outline the procedures for extracting the installed software from a running container of the image and creating a portable ROS 2 package out of the install space.
Because this is being done based on a Linux-based image we recommend doing the following on a Linux host as well; the instructions below will assume that and be using a lot of \*NIX command line tools to set up the deliverable package.

Note that the Linux Dockerfile does not use the symlink install scheme from ament; this is required for the built workspace to be portable.

#### 1. Build the image

As noted above you'll need to use this directory as your working directory. A command something like:

    $ docker build -t ihmc_ros1_bridge:ardent -f src/rosdocker/linux/Dockerfile .
    
Should work. Note that the Linux Dockerfile is set up to build for the bridge by default and the bridge is a tremendously demanding compilation process that requires a substantial amount of RAM. If you receive build failures try
examining the Dockerfile and uncommenting the makefile variables directive to enforce single threaded compilation.

#### 2. Run the image

If you used the command above you should have a tagged image for the Ardent ROS 1 Bridge. We're going to start a container named `ros1_bridge` using that image:

    $ docker run --name ros1_bridge -it ihmc_ros1_bridge:ardent
    
Because the image doesn't have a proper entrypoint for a long-running process, this will start a container with a shell attached to it; you'll need to leave this terminal alone while completing the process so the image stays running while you extract the software. The rest of this process will be done from an additional terminal on the host system (not attached to the Docker container)

#### 3. Extract the installed software from the running image

Now that we have a running container, we need to copy out the compiled ROS 1 Bridge to our host machine so that we can package it. You may want to do this in a working directory that's not in the Git working copy so that you don't acccidentally commit a ton of binaries.
Once you've changed working directories, Docker provides a handy copy command for things this:

    $ docker cp ros1_bridge:/root/ihmc_ros1_bridge .
    
Depending on how your Docker installation is set up (especially if it's set up to run via `sudo`), the extracted files may be owned by `root` or `docker`.
Change this to your local unpriveleged user with a recursive `chown`, for example something like: `sudo chown -R shadylady:shadylady ihmc_ros1_bridge/`

#### 4. Clean-up the package before making a tarball

The cleanup process is pretty straight forward; we need to rewrite some Python shebangs to be explicit and we need to exclude a few files from the tarball. The first is accomplished by just doing a find/replace in a few files.

First let's `cd ihmc_ros1_bridge`.

1. Cleaning up shebangs: What we're doing here is looking for python 3 shebangs and tidying them up. One way to do this is to grep for all shebangs in the `install` space: `grep -iR \#\! install/`.
Any python shebangs need to be of this form: `#!/usr/bin/env python3`. If you encounter any Python shebangs that are NOT of that form, you need to modify those files in the `install` space.

2. Excluding some files from the tarball: There are many ways to do this, we're going to use an excludes file. `cd ..` to get out of the extracted ros1_bridge directory and make a file called `tar-excludes` with the following contents:
    
    ```
    .egg-info
    SOURCES.txt
    ```
    
We'll pass this file in to the tar command and it will filter out the files we don't want.

#### 5. Making the tarball

You should now be in a working directory containing a child directory called `ihmc_ros1_bridge` that was extracted from the running container, and a `tar-excludes` file to filter the files out of the .tarball. The following tar command will make a compressed
tarball using the naming scheme that we use for uploading to Bintray; if you need a new naming scheme then feel free. The tarball command is complicated but the gist is that we're creating a compressed tar archive of *only* the `install` space, which we use
a transformation to "rename" inside the archive so that it doesn't extract as a directory named `install`. Replace the `<version number>` placeholder with the appropriate version number:

    $ tar -cvjSf ihmc-ros-ardent-ros1-bridge-ubuntu-xenial-<version number>.tar.bz2 -X tar-excludes --transform='s/ihmc_ros1_bridge\/install/ihmc_ros1_bridge/g' ihmc_ros1_bridge/install
    
Congratulations! You're now done. You can distribute this tarball to other Ubuntu Xenial + ROS Ardent users if they need access to our ROS 1 Bridge.


