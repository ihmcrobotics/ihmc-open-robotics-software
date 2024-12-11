# IHMC-External Control

**NOTE: We currently ONLY support Ubuntu 22.04. Other versions of Ubuntu, and Windows / macOS operating systems are not supported and are almost certain to break.**

After reading this and installing, be sure to read [ARCHITECTURE.md](ARCHITECTURE.md) and [CONTRIBUTING.md](CONTRIBUTING.md) for more information on how the code in this repository is structured, and how to contribute effectively.

## Contents

* [Overview](#overview)
* [Dependencies](#dependencies)
* [Install](#install)
* [Development](#development)
* [Troubleshooting](#troubleshooting)
    * [Failed Docker container creation](#failed-docker-container-creation)
    * [Annoying permissions issues in generated-java](#annoying-permissions-issues-in-generated-java)

## Overview

* This project contains a thin C++ wrapper to allow setting up calls to a custom controller on the C++ side, and then interfacing with it from Java
* We create JNI bindings for the C++ wrapper library

## Dependencies

* Docker (check with `docker --version`). You'll want to run `docker` commands without having to prefix `sudo` all the
  time, follow the instructions [here](https://askubuntu.com/questions/477551/how-can-i-use-docker-without-sudo).

## Install

Starting from a typical IHMC setup consisting of a `repository-group` workspace
with `ihmc-open-robotics-software`, perform the following steps:

1. Clone this repository into `repository-group`.

(NOTE: you may need to `chmod +x <script>` to run the following scripts)

2. From the command line or your IDE terminal, navigate to `external-control` and run the
   script `./createDockerImage.sh` to create the docker image used for isolated building of the dependencies.
3. From the command line or your IDE terminal, navigate to `external-control` and run the
   script `./generateJavaBindingsCI.sh` (if building for development) or `./generateJavaBindingsRelease.sh` (if building
   for robot deployment, this is in-development). This builds the C++ wrapper in the docker container, and
   is subsequently used to generate the Java bindings. The native Java bindings are generated inside the docker
   container, and then everything is copied (including the required shared libraries) from inside the docker container
   to the `src/main/resources` directory of this project to be used by IHMC's native library loader. *NOTE: this step
   may take a short while*.

## Development

* If using JetBrains IDEs (IntelliJ for Java, CLion for C++), they will not play nice if they are opened in the same
  directory. To combat this, open IntelliJ in the overall `repository-group` workspace, and CLion in
  the `ihmc-open-robotics-software` directory. This way, the `.idea` directories are separate and won't conflict with each
  other.
* When developing the wrapper in CLion, set up a Docker CMake toolchain to build the project, the image must be set to
  the image created when `createDockerImage.sh` was
  run: https://www.jetbrains.com/help/clion/clion-toolchains-in-docker.html
* When working in C++, please use [this style guide](IHMC-MPC-CodeStyle.xml). You can import the XML into CLion so that
  you can auto-format your code.

## Troubleshooting

### Failed Docker container creation

If running `createDockerImage.sh` fails for any reason, it is highly likely that docker will cache some image
layers that are incorrect and will cause future attempts at image creation to fail.

Before running `createDockerImage.sh` again:

1. run `docker image remove ihmc-external-wrapper:0.1` (or whatever version of the wrapper you're using)
2. run `docker container prune` and `docker image prune`
3. verify that `docker image list -a` is empty, this means all the cached images have been deleted
4. run `createDockerImage.sh` again


### Annoying permissions issues in `generated-java`

Due to the way our shell scripts build the wrapper and generate the bindings, it is quite likely that the `src/main/generated-java`
and `src/main/resources` directories will be owned by `root`. This is annoying because it means that you can't delete or rollback
the changes in git. To fix this, run this from the project root:

```bash
sudo chown -R $USER:$USER ihmc-mpc-core ihmc-mpc-nadia
```
