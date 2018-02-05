# IHMC Open Robotics Software

### 0.11 Build Info
Build 4570
https://bamboo.ihmc.us/browse/LIBS-IHMCOPENROBOTICSSOFTWARE-1067

### Tested Platforms

#### Robots

- Atlas
   * This release is fully tested on Atlas hardware. See 0.11 Release Notes for detailed results.

**This release DOES NOT support the Valkyrie hardware platform, as we do not have hardware to test on.**

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
      url  "http://dl.bintray.com/ihmcrobotics/maven-release" // IHMC Code releases
   }
   maven {
      url  "http://dl.bintray.com/ihmcrobotics/maven-vendor" // Third-party libraries that we have vendored for various reasons
   }

   /* You will also need to add either jcenter() or mavenCentral() or both, depending on your preference */
}
```

### Developing with *IHMC Open Robotics Software* from source

#### Requirements
*IHMC Open Robotics Software* uses the [Gradle](https://gradle.org) build system, and requires JDK 8 with JavaFX. We also strongly suggest an IDE, either Eclipse Mars.1
or IntelliJ IDEA 2017.3+ (Ultimate or Community is fine). Currently, we require **Gradle 4.1+**. We provide a versioned [Gradle wrapper](https://docs.gradle.org/current/userguide/gradle_wrapper.html)
for getting started quickly. The Gradle wrapper will always reflect the minimum version of Gradle required to build the software; if we adopt features only present
in newer versions of Gradle as they are release we will update the wrapper. You can also install Gradle system-wide (local installation):

Installing Gradle: https://gradle.org/install/

#### IDE Support
Our Gradle models are tested in IntelliJ IDEA 2017.3+ (both Community and Ultimate) with the Gradle plugin.
Eclipse Oxygen+ or higher with the Buildship plugin. The Buildship plugin is bundled with the Eclipse IDE for Java Developers (but *not* Java EE Developers). It can always be manually installed to any version of Eclipse using the [installation instructions](https://github.com/eclipse/buildship/blob/master/docs/user/Installation.md).

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
$ ./gradlew compositeTask -PtaskName=publishToMavenLocal -PdepthFromWorkspaceDirectory=0 -PpublishMode=LOCAL
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

#### Depending directly on the source
For *IHMC Open Robotics Software* and [ihmc-build](https://github.com/ihmcrobotics/ihmc-build) to work correctly when depending directly on the source, your
project hierarchy needs to take a particular form.

1. In your system home folder (or C:/ drive in Windows), create a directory called `ihmc-workspace`.
1. Initialize the `ihmc-workspace` directory as an IHMC repository group using the "Convert an existing project group" instructions in [this README] (https://github.com/ihmcrobotics/repository-group).
1. This is also the directory where you'll put any of your own projects that need to depend
on IHMC source code. Your directory structure should look something like:

```
repository-group
├── my-project-a
│   └── build.gradle
│   └── gradle.properties
│   └── settings.gradle
├── my-project-b
│   └── ...
├── ihmc-open-robotics-software
│   └── acsell
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

If this is set up correctly, you can either [apply the `ihmc-build` plugin](https://github.com/ihmcrobotics/ihmc-build)
and use the dependency resolver methods exposed by the build extension, or you can manually identify dependencies on projects using the normal Gradle syntax for
project dependencies. A sample build.gradle dependency block:

```gradle
dependencies {
  compile project(':ihmc-open-robotics-software:ihmc-java-toolkit') // normal Gradle way of doing things
}

/* OR */

mainDependencies {
  compile group: "us.ihmc", name: "ihmc-java-toolkit", version: "source" // ihmc-build way of doing things
}
```

## Support

Chat support is provided via the [IHMC Robotics Slack](ihmcrobotics.slack.com) on the #help-desk channel.

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
