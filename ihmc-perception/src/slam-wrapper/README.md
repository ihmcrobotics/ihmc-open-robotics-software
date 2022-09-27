# slam-wrapper with Java bindings

## Overview

### Goal:
Be able to use SLAM elements of C++ library, GTSAM, from Java.

### Solution:
Use javacpp/JNI to create Java-to-C++ mappings for access to GTSAM. javacpp can automatically generate JNI code so we don't have to write it all manually ourselves. Generating mappings for GTSAM itself is a lot of work; to address this, we abstract out the functionality we need into a separate small C++ project which internally uses GTSAM.

Our C++ wrapper only exposes functions which accept primitive types as arguments (ie a float array which represents a GTSAM object, perhaps gtsam::Pose3, for example). This way, our JNI mappings are extremely simple and minimal. The method of use is to pack your data on the Java-end into a float array, pass it to the native JNI function, and the C++ wrapper will unpack and construct the appropriate datatypes on the other end.

### Main additions:
- Create a small C++ wrapper to abstract complex data types of GTSAM. I.e. functions which only take primitive types
- Create build scripts which utilize javacpp to generate JNI bindings for said C++ wrapper

Sourcesets/directory structure:
```
ihmc-perception/
  src/
    slam-wrapper/
      cpp/
        - our C++ wrapper for GTSAM lives here as well as helper build scripts
      generated-java/
        - our generated Java files from our build scripts in ../cpp live here
      java/
        - our javacpp helper classes live here
      resources/
        - any shared libraries (.so/dll/dylib) live here
```

## Supported platforms
- GNU/Linux distro with glibc >= 2.31 (ie Ubuntu 20.04 or newer; will work on others)

## Building/generating the Java source set
### Requirements
- Docker (for now)
- A recent Linux distro

To build slam-wrapper and generate the Java source bindings automatically, run `generate-java-mappings-docker.sh` in the `cpp` directory.

The build files will end up in `cpp/build`. The binaries and source code will be copied to their respective locations:
- Java generated from javacpp gets placed in `generated-java`
- Shared libraries (.so files) get placed in `resources`