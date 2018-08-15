---
title: Installing Gradle
---

### Windows 


Gradle has no automated installer. Windows users will need to follow the installation instructions for Gradle from their documentation <https://docs.gradle.org/current/userguide/installation.html>.

The installation involves editing your user PATH variable; this is only necessary if you wish to run Gradle from the command line. Often times, IDEs will allow you to point to where Gradle is installed. It may be sufficient to simply unpack the .zip archive if you plan to use an IDE.


### Ubuntu

Up-to-date versions of Gradle are available from a Launchpad PPA. The version of Gradle available without the PPA is woefully out of date. You will need to add the following PPA: <https://launchpad.net/~cwchien/+archive/ubuntu/gradle>. All put together, this looks like:

```bash
$ sudo add-apt-repository ppa:cwchien/gradle
$ sudo apt-get update
$ sudo apt-get install gradle
```

### OSX 

Gradle is available from [Homebrew](http://brew.sh/). Once you install homebrew, you can install Gradle as follows:

```bash
$ brew update
$ brew install gradle
```

