# IHMC Open Robotics Software

## Current Continuous Integration Status

![develop, fast](https://github.com/ihmcrobotics/ihmc-open-robotics-software/actions/workflows/gradle-test-fast.yml/badge.svg?branch=develop) Current develop status

![develop, fast](https://github.com/ihmcrobotics/ihmc-open-robotics-software/actions/workflows/gradle-test-fast.yml/badge.svg?branch=0.14.0-240126) Current release status

## Robots

This software has been used to power a wide variety of humanoid robots. This includes
- [Nadia](https://www.ihmc.us/nadia-humanoid/)
- [Valkyrie](https://www.nasa.gov/wp-content/uploads/2023/06/r5-fact-sheet.pdf)
- [Alex](https://boardwalkrobotics.com/)

## Licensing

All of the software in *IHMC Open Robotics Software* is licensed under the Apache 2.0 license.

## Developing with *IHMC Open Robotics Software* from source

*IHMC Open Robotics Software* uses the [Gradle](https://gradle.org) build system, and requires JDK 17. 
We recommend working in IntelliJ.


See the following tutorials for installing and using IHMC Open Robotics Software:
- [Ubuntu (22.04, 24.04)](https://ihmcrobotics.atlassian.net/wiki/external/ZTE1Y2NkMzA2MTUyNGIzOGI0NjkyZTMwYzgwZDY3MGU)
- [Windows (10, 11)](https://ihmcrobotics.atlassian.net/wiki/external/NDQ2ZDE2MWRkOGYxNDY4OTk3N2M5NWRiNDU2MGNkMTI)

Arch Linux will work fine for development.
Other GNU/Linux distros will likely work, however largely untested.
macOS is partially supported, but incomplete at this time.

To get set up, use our public Confluence pages:
https://ihmcrobotics.atlassian.net/wiki/spaces/PUBLIC/overview

## Other IHMC Libraries

*IHMC Open Robotics Software* both depends on, and is depended on by, a large ecosystem of IHMC robotics libraries. Below is a representative (but not exhaustive) list of related projects used by this repository.

### Simulation & Visualization
- **Simulation Construction Set 2 (SCS2)**  
  Modern simulation environment with built-in analysis tools  
  https://github.com/ihmcrobotics/simulation-construction-set-2  
  ![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/main-gradleCI-build.yml/badge.svg)

- **Simulation Construction Set (legacy)**  
  Deprecated simulation engine still used for base visualization

### Math, Geometry, & Dynamics
- **Euclid**  
  Java vector and geometry library with common 3D structures  
  https://github.com/ihmcrobotics/euclid  
  ![buildstatus](https://github.com/ihmcrobotics/euclid/actions/workflows/gradle-test.yml/badge.svg)

- **Mecano**  
  Rigid-body dynamics library built on Euclid and EJML  
  https://github.com/ihmcrobotics/mecano  
  ![buildstatus](https://github.com/ihmcrobotics/mecano/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC Matrix Library**  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-matrix-library/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC Convex Optimization**  
  Collection of algorithms for solving convex optimization problems  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-convex-optimization/actions/workflows/gradle-test.yml/badge.svg)

### Data, Logging, & Analysis
- **IHMC YoVariables**  
  Core data structures for time-series tracing and SCS analysis  
  https://github.com/ihmcrobotics/ihmc-yovariables  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-yovariables/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC Robot Data Logger**  
  Application for logging YoVariables  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-robot-data-logger/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC Log Tools**  
  ![buildstatus](https://github.com/ihmcrobotics/log-tools/actions/workflows/gradle-test.yml/badge.svg)

### Perception & State Estimation
- **JOctoMap**  
  Java implementation of OctoMap  
  https://github.com/ihmcrobotics/joctomap  
  ![buildstatus](https://github.com/ihmcrobotics/joctomap/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC EKF**  
  ![buildstatus](https://github.com/ihmcrobotics/ekf/actions/workflows/gradle-test.yml/badge.svg)

### Real-Time, Communication, & Middleware
- **IHMC Realtime**  
  Soft real-time Java threading on Linux using RT_PREEMPT  
  https://github.com/ihmcrobotics/ihmc-realtime  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-realtime/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC EtherCAT Master**  
  Java EtherCAT master built on IHMC Realtime and SOEM  
  https://github.com/ihmcrobotics/ihmc-ethercat-master  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-ethercat-master/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC ROS 2 Library**  
  Lightweight Java implementation of the ROS 2 communication protocol  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-java-ros2-communication/actions/workflows/gradle.yml/badge.svg)

- **IHMC PubSub**  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-pub-sub-group/actions/workflows/build-natives.yml/badge.svg)  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-pub-sub-group/actions/workflows/run-gradle-test-all-platforms.yml/badge.svg)  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-pub-sub-group/actions/workflows/run-gradle-test-linux-self-hosted.yml/badge.svg)

### UI, Graphics, & Media
- **IHMC JavaFX Toolkit**  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-javafx-toolkit/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC Graphics Description**  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-graphics-description/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC Video Codecs**  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-video-codecs/actions/workflows/gradle-test.yml/badge.svg)

### Utilities
- **IHMC Commons**  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-commons/actions/workflows/gradle-test.yml/badge.svg)

- **IHMC Native Library Loader**  
  ![buildstatus](https://github.com/ihmcrobotics/ihmc-native-library-loader/actions/workflows/gradle-test.yml/badge.svg)

You can find these and many other IHMC projects at:  
https://github.com/ihmcrobotics

## ROS APIs
We provide a ROS 2 API for many of the core components in our software stack. 
You can find the .msg definitions for use in your own projects in this project's ihmc-interfaces folder.

## Building .jars
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

## Creating a project
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
* James Foster (jfoster@ihmc.org)
* Dexton Anderson (danderson@ihmc.org)
* Luigi Penco (lpenco@ihmc.org)
* Nick Kitchel (nkitchel@ihmc.org)
