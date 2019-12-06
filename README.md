# IHMC Open Robotics Software

**Compile:** ![Compile](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-IHMCOPENROBOTICSSOFTWARE)
**Test (3000+ tests):** ![Test](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-IHMCOPENROBOTICSSOFTWAREFAST)

 [ ![Download](https://api.bintray.com/packages/ihmcrobotics/maven-release/atlas/images/download.svg?version=0.12.0) ](https://bintray.com/ihmcrobotics/maven-release/atlas/0.12.0/link) <-- Download from Bintray

### Tested Platforms

#### Robots

- Atlas
- Valkyrie

#### Developers

We test all of our software on OSX, Windows, and Ubuntu. It is likely to work on other platforms but
not necessarily tested.

### Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Licensing
All of the software in *IHMC Open Robotics Software* is licensed under the Apache 2.0 license.

## Getting Started

### Using IHMC Open Robotics Software .jar releases with Maven/Gradle
The release .jars for the various *IHMC Open Robotics Software* packages are hosted on Bintray. You can browse the release packages at https://bintray.com/ihmcrobotics/maven-release.
Instructions for adding the Maven repository and identifying the artifacts can also be found on Bintray for each package.

At a minimum, you will need to have the following repositories declared in your build script to use *IHMC Open Robotics Software* .jars:

```gradle
repositories {
   maven {
      url  "https://dl.bintray.com/ihmcrobotics/maven-release" // IHMC Code releases
   }
   maven {
      url  "https://dl.bintray.com/ihmcrobotics/maven-vendor" // Third-party libraries that we have vendored for various reasons
   }

   /* You will also need to add either jcenter() or mavenCentral() or both, depending on your preference */
}
```

### Developing with *IHMC Open Robotics Software* from source

#### Requirements
*IHMC Open Robotics Software* uses the [Gradle](https://gradle.org) build system, and requires JDK 8 with JavaFX. We also strongly suggest an IDE, either Eclipse
or IntelliJ (Ultimate or Community is fine). Currently, we require **Gradle 4.10+**.

Installing Gradle: https://gradle.org/install/

### Companion Software

#### Other IHMC Libraries
IHMC Open Robotics Software both depends on and is depended on by many other IHMC Robotics Libraries. A small sampling of our other software includes:

- Simulation Construction Set, our own simulation environment with built-in analysis tools: https://github.com/ihmcrobotics/simulation-construction-set
- Euclid, an alternative vector/geometry library for Java with support for additional structures common in 3D geometry without needing vecmath or Java3D: https://github.com/ihmcrobotics/euclid
- Mecano, a rigid-body dynamics library built on top of Euclid and EJML: https://github.com/ihmcrobotics/mecano
- IHMC YoVariables, our core data structures tools that enable the time-series tracing and analysis of Simulation Construction Set: https://github.com/ihmcrobotics/ihmc-yovariables
- JOctoMap, a Java implementation of OctoMap: https://github.com/ihmcrobotics/joctomap
- IHMC Realtime, a library for enabling soft real-time threading for Java on Linux using the RT_PREEMPT patches: https://github.com/ihmcrobotics/ihmc-realtime
- IHMC EtherCAT Master, a Java library using IHMC Realtime and Simple Open EtherCAT Master (SOEM) that makes it simple to start a software EtherCAT Master and pure Java data structures that map to EtherCAT Slave defintions: https://github.com/ihmcrobotics/ihmc-ethercat-master

You can find all of our other repositories as well as the ones above at https://github.com/ihmcrobotics

#### ROS API's
We provide a native ROS 2 API for many of the core components in our software stack. You can find the .msg definitions for use in your own projects here: https://github.com/ihmcrobotics/ihmc_interfaces

We have ROS 1 support via the ROS 2 `ros1_bridge` package. You can find the ROS 1 message definitions and instructions on using the ROS 1 Bridge here: https://github.com/ihmcrobotics/ihmc_msgs

#### IDE Support
Our Gradle models are tested in IntelliJ IDEA 2018 (both Community and Ultimate) with the Gradle plugin.
Eclipse 2018.09+ or higher with the Buildship plugin. The Buildship plugin is bundled with the Eclipse IDE for Java Developers (but *not* Java EE Developers). It can always be manually installed to any version of Eclipse using the [installation instructions](https://github.com/eclipse/buildship/blob/master/docs/user/Installation.md).

#### Building .jars
*IHMC Open Robotics Software* is pre-configured for generating Maven publications. You can publish directly from the source code right in to your local Maven
repository, e.g. the `$HOME/.m2` directory. These builds will be tagged with a build "version" of `"LOCAL"` instead of an incrementing version number.

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

In this example we'll have a compile-time dependency of the locally built Simulation Construction Set project. In the `build.gradle` of the project you wish to
have link against Simulation Construction Set:

```gradle
repositories {
  mavenLocal()
  <your other repositories>
}

dependencies {
  compile group: "us.ihmc", name: "simulation-construction-set", version: "LOCAL", changing: true
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
│   └── build.gradle
│   └── gradle.properties
│   └── settings.gradle
├── my-project-b
│   └── ...
├── ihmc-open-robotics-software
│   └── atlas
│   └── common-walking-control-modules
│   └── ...
├── my-multi-project-c
│   └── subproject-a
│   │  └── build.gradle
│   └── subproject-b
│      └── build.gradle
├── ...
├── build.gradle
├── gradle.properties
└── settings.gradle
```

If this is set up correctly, you will have applied the [`ihmc-build` plugin](https://github.com/ihmcrobotics/ihmc-build)
and use the dependency resolver methods exposed by the build extension. Alternatively, you can manually identify dependencies on projects using the normal Gradle syntax for
project dependencies. A sample build.gradle dependency block:

```gradle
/* Normal Gradle way */
dependencies {
  compile project(':ihmc-open-robotics-software:ihmc-java-toolkit')
  testCompile project(':ihmc-open-robotics-software:ihmc-java-toolkit-test')
}

/* ihmc-build way */
mainDependencies {
  compile group: "us.ihmc", name: "ihmc-java-toolkit", version: "source"
}
testDependencies {
  compile group: "us.ihmc", name: "ihmc-java-toolkit-test", version: "source"
}
```

## Maintainers

* Jerry Pratt (jpratt@ihmc.us)
* Peter Neuhaus (pneuhaus@ihmc.us)
* Doug Stephen (dstephen@ihmc.us)
* Sylvain Bertrand (sbertrand@ihmc.us)
* Duncan Calvert (dcalvert@ihmc.us)
* Stephen McCrory (smcrory@ihmc.us)
* Robert Griffin (rgriffin@ihmc.us)
* Georg Wiedebach (gwiedebach@ihmc.us)
* Inho Lee (ilee@ihmc.us)
* Daniel Duran (dduran@ihmc.us)
* John Carff (jcarff@ihmc.us)
