# IHMC Open Robotics Software

**Compile:** ![Compile](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-IHMCOPENROBOTICSSOFTWARE)
**Test (3000+ tests):** ![Test](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-IHMCOPENROBOTICSSOFTWAREFAST)

### Tested Platforms

#### Robots

- DRC-Atlas
- Valkyrie
- [Nadia](https://boardwalkrobotics.com/Nadia.html)

#### Developers

Our developers are a mix of Windows and Linux users. We officially support:
- Windows 10/11
- Ubuntu 20.04+

Other GNU/Linux distros will likely work, however largely untested. macOS is partially supported, but incomplete at this time.

### Branches
This repository uses the git-flow branching model. 
You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Licensing
All of the software in *IHMC Open Robotics Software* is licensed under the Apache 2.0 license.

## Getting Started

### Developing with *IHMC Open Robotics Software* from source

#### Requirements
*IHMC Open Robotics Software* uses the [Gradle](https://gradle.org) build system, and requires JDK 17. 
We also strongly suggest an IDE, either Eclipse
or IntelliJ (Ultimate or Community is fine). Currently, we require **Gradle 7.5.1+**.

Installing Gradle: https://gradle.org/install/

### Companion Software

#### Other IHMC Libraries
IHMC Open Robotics Software both depends on and is depended on by many other IHMC Robotics Libraries. 
A small sampling of our other software includes:

- Simulation Construction Set 2, our own simulation environment with 
built-in analysis tools: https://github.com/ihmcrobotics/simulation-construction-set-2
- Euclid, an alternative vector/geometry library for Java with support for additional structures 
common in 3D geometry without needing vecmath or Java3D: https://github.com/ihmcrobotics/euclid
- Mecano, a rigid-body dynamics library built on top of Euclid and EJML: https://github.com/ihmcrobotics/mecano
- IHMC YoVariables, our core data structures tools that enable the time-series tracing 
and analysis of Simulation Construction Set: https://github.com/ihmcrobotics/ihmc-yovariables
- JOctoMap, a Java implementation of OctoMap: https://github.com/ihmcrobotics/joctomap
- IHMC Realtime, a library for enabling soft real-time threading for Java on Linux using the 
RT_PREEMPT patches: https://github.com/ihmcrobotics/ihmc-realtime
- IHMC EtherCAT Master, a Java library using IHMC Realtime and Simple Open EtherCAT Master (SOEM) 
that makes it simple to start a software EtherCAT Master and pure Java data structures that map to 
EtherCAT Slave defintions: https://github.com/ihmcrobotics/ihmc-ethercat-master

You can find all of our other repositories as well as the ones above at https://github.com/ihmcrobotics

#### ROS API's
We provide a native ROS 2 API for many of the core components in our software stack. 
You can find the .msg definitions for use in your own projects in this project's ihmc-interfaces folder.

We have ROS 1 support via the ROS 2 `ros1_bridge` package. 
You can find the ROS 1 message definitions and instructions on using the 
ROS 1 Bridge here: https://github.com/ihmcrobotics/ihmc_msgs

#### IDE Support
Our Gradle models are tested in IntelliJ IDEA (both Community and Ultimate) with the Gradle plugin.
Eclipse 2021 or higher with the Buildship plugin. The Buildship plugin is bundled with the 
Eclipse IDE for Java Developers (but *not* Java EE Developers). 
It can always be manually installed to any version of Eclipse using the 
[installation instructions](https://github.com/eclipse/buildship/blob/master/docs/user/Installation.md).

#### Building .jars
*IHMC Open Robotics Software* is pre-configured for generating Maven publications. 
You can publish directly from the source code right in to your local Maven
repository, e.g. the `$HOME/.m2` directory. These builds will be tagged with a 
build "version" of `"LOCAL"` instead of an incrementing version number.

An example workflow for developing against a local clone of the software:

1. Clone *IHMC Open Robotics Software*
2. Make modifications
3. Publish to your local `$HOME/.m2` repository

**To publish jars to your local Maven repository:**
```bash
$ cd /path/to/ihmc-open-robotics-software
$ gradle publishAll -PcompositeSearchHeight=0
```

**To depend on the jars in your local Maven repository:**

In this example we'll have a compile-time dependency of the locally built 
Simulation Construction Set project. In the `build.gradle` of the project you wish to
have link against Simulation Construction Set:

```gradle
repositories {
  mavenLocal()
  <your other repositories>
}

dependencies {
  api("us.ihmc:simulation-construction-set:LOCAL") {
     changing = true
  }
}
```  

#### Creating a project
To create a project that uses *IHMC Open Robotics Software*, your
project hierarchy needs to take a particular form.

First be sure you have completed the section above titled "Clone repositories".

Next, create your project folder:

```
mkdir -p src/ihmc/my-project-a
```

Follow the project setup tutorial at https://github.com/ihmcrobotics/ihmc-build#quick-project-setup.

Your directory structure should now look something like:

```
src/ihmc
├── my-project-a
│   └── build.gradle.kts
│   └── gradle.properties
│   └── settings.gradle.kts
├── my-project-b
│   └── ...
├── ihmc-open-robotics-software
│   └── atlas
│   └── common-walking-control-modules
│   └── ...
├── my-multi-project-c
│   └── subproject-a
│   │  └── build.gradle.kts
│   └── subproject-b
│      └── build.gradle.kts
├── ...
├── build.gradle.kts
├── gradle.properties
└── settings.gradle.kts
```

If this is set up correctly, you will have applied the [`ihmc-build` plugin](https://github.com/ihmcrobotics/ihmc-build)
and use the dependency resolver methods exposed by the build extension. 
Alternatively, you can manually identify dependencies on projects using the normal Gradle syntax for
project dependencies. A sample build.gradle dependency block:

```build.gradle.kts
/* Normal Gradle way */
dependencies {
  api(project(":ihmc-open-robotics-software:ihmc-java-toolkit"))
  testApi(project(":ihmc-open-robotics-software:ihmc-java-toolkit-test"))
}

/* ihmc-build way */
mainDependencies {
  api("us.ihmc:ihmc-java-toolkit:source")
}
testDependencies {
  api("us.ihmc:ihmc-java-toolkit-test:source")
}
```

## Maintainers

* Sylvain Bertrand (sbertrand@ihmc.org)
* Duncan Calvert (dcalvert@ihmc.org)
* Stephen McCrory (smcrory@ihmc.org)
* Robert Griffin (rgriffin@ihmc.org)
* Bhavyansh Mishra (bmishra@ihmc.org)
* James Foster (jfoster@ihmc.org)
* Dexton Anderson (danderson@ihmc.org)
* Luigi Penco (lpenco@ihmc.org)
* Nick Kitchel (nkitchel@ihmc.org)
