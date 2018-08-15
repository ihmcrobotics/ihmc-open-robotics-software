---
title: Installing Java
---

### Windows 7+

We support Windows 7/8/10.
 
You will need to install the JDK from Oracle, which can be found here: <http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html>

Our software is compatible with Java 8u111 or higher.


### Ubuntu


We currently support Ubuntu 14.04 LTS with Oracle Java 8 or Ubuntu 16.04 using either OpenJDK 8 or Oracle JDK 8.

We suggest minor version u111 or higher because of certificate authority complications; using lower versions might cause problems for the IHMC build scripts with authenticating against certain servers to download dependencies.

The reason we require Oracle JDK 8 on Ubuntu 14.04 LTS is because we utilize JavaFX, and the current OpenJDK PPA for Ubuntu 14.04 does not have an OpenJFX build. The 16.04 repositories come with OpenJDK 8 out of the box as well as OpenJFX


* Installing OpenJDK 8 on Ubuntu 16.04


Ubuntu ships with OpenJDK 8 in its repositories. We suggest also installing the source attachments and Javadocs for easy code navigation in your IDE. To install:

```bash
$ sudo apt-get update
$ sudo apt-get install openjdk-8-jdk openjdk-8-doc openjdk-8-source openjfx
```


* Installing Oracle Java 8 on Ubuntu 14.04 or Ubuntu 16.04


Oracle Java 8 can be tapped via a PPA for usage on Ubuntu 14.04. To use Oracle Java 8:

```bash
$ sudo add-apt-repository ppa:webupd8team/java
$ sudo apt-get update
$ sudo apt-get install oracle-java8-installer oracle-java8-set-default
```

More information about what these commands are doing can be found here: <http://www.webupd8.org/2012/09/install-oracle-java-8-in-ubuntu-via-ppa.html>


### OSX

We currently test all software on OS X 10.12 Sierra***\**** via the Oracle JDK, version 8u111 or higher: <http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html>

***\****The software may work on OS X versions as far back as Mavericks, but we do not currently actively build/test on Mavericks/Yosemite/El Capitan machines.


* Cleaning up Apple Java/Resolving Java3D Errors


Before Java 7, Apple used to provide a Java distribution themselves. A lot of existing software still relies on the Apple Java 6 distribution, and many OS X machines that have been around for a while will probably have had this old Java distribution installed at some point.

This can cause problems with our software. The Apple Java distribution bundled an older version of the Java3D library, and this old version of Java3D will take precedence over the newer version that we bundle with our software. It can lead to errors as it is not API compatible with newer versions of Java3D. There are three Java3D libraries that need to be removed. Delete the following files (**_if they exist_**):

    /System/Library/Java/Extensions/j3dcore.jar
    /System/Library/Java/Extensions/j3dutils.jar
    /System/Library/Java/Extensions/vecmath.jar

***NOTE:*** If you are using OS X El Capitan or newer, these libraries are in directories that are protected by [System Integrity Protection (SIP)](https://en.wikipedia.org/wiki/System_Integrity_Protection). You will need to disable SIP. It can be re-enabled later if you so choose. Instructions for disabling SIP can be found here: <http://www.macworld.com/article/2986118/security/how-to-modify-system-integrity-protection-in-el-capitan.html>



