# IHMC Open Robotics Software

## Minutiae

### Tested Platforms
We test all of our software on OS X 10.11 El Capitan, Windows 7/8/10, and Ubuntu 14.04 LTS Desktop and Server.

### Branches
This repository uses the git-flow branching model. You can find more about git-flow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Licensing
Individual projects all have their own license information. We use a combination of Apache 2.0 and GPLv3. Consult the license file in each project for more information.

## Getting Started

### Using IHMC Open Robotics Software .jar releases with Maven/Gradle
The release .jars for the various *IHMCOpenRoboticsSoftware* packages are hosted on Bintray. You can browse the release packages at https://bintray.com/ihmcrobotics/ihmc-robotics-releases. Instructions for adding the Maven repository and identifying the artifacts can also be found on Bintray for each package.

### Developing with IHMC Open Robotics Software from source

#### Requirements
*IHMCOpenRoboticsSoftware* uses the [Gradle](https://gradle.org) build system, and requires JDK 7 or 8. We also strongly suggest an IDE, either Eclipse Mars.1 or IntelliJ IDEA 14+ (Ultimate or Community is fine). Currently, we require **Gradle 2.10+**. We provide a versioned [Gradle wrapper](https://docs.gradle.org/current/userguide/gradle_wrapper.html) for getting started quickly. The Gradle wrapper will always reflect the minimum version of Gradle required to build the software; if we adopt features only present in newer versions of Gradle as they are release we will update the wrapper. You can also install Gradle system-wide:

* *OS X*: The latest Gradle is available via [Homebrew](https://github.com/homebrew/homebrew)
* *Ubuntu 14.04*: The version pre-packaged with Ubuntu is woefully out of date. You will need to add the following PPA: https://launchpad.net/~cwchien/+archive/ubuntu/gradle
* *Windows*: Consult the official Gradle documentation installation instructions.

#### IDE Support
Our Gradle models are tested in IntelliJ IDEA 14+ (both Community and Ultimate) with the built-in Gradle integration, and Eclipse Mars.1 or higher with the Buildship plugin. The Buildship plugin is bundled with the Eclipse IDE for Java Developers as of Mars.1 (but *not* Java EE Developers), or it can be downloaded from the Eclipse Marketplace in other versions of Eclipse.

#### Building .jars
IHMCOpenRoboticsSoftware is pre-configured for generating Maven publications. You can publish directly from the source code right in to your local Maven repository, e.g. the `$HOME/.m2` directory. These builds will be tagged with a build "version" of `"LOCAL"` instead of an incrementing version number.

An example workflow for developing against a local clone of the software:

1. Clone IHMCOpenRoboticsSoftware
2. Make modifications
3. Publish to your local `$HOME/.m2` repository

**To publish jars to your local Maven repository:**  
```bash
$ cd <IHMCOpenRoboticsSoftware directory>
$ ./gradlew publishAllToMavenLocal
```

**To depend on the jars in your local Maven repository:**

In this example we'll have a compile-time dependency of the locally built SimulationConstructionSet project. In the `build.gradle` of the project you wish to have link against SimulationConstructionSet:

```gradle
repositories {
  mavenLocal()
  <your other repositories>
}

dependencies {
  compile group: 'us.ihmc', name: 'SimulationConstructionSet', version: 'LOCAL', changing: true
}
```  

#### Depending directly on the source
For the IHMCOpenRoboticsSoftware and [ihmc-build](https://github.com/ihmcrobotics/ihmc-build) to work correctly when depending directly on the source, the Gradle project hierarchy needs to take a particular form. Let's assume you have a directory structure such as:

```
.<your Gradle root project>
├── build.gradle
├── settings.gradle
├── ProjectA
├── ProjectB
├── IHMCOpenRoboticsSoftware
│   └── Acsell
│   └── Atlas
│   └── CommonWalkingControlModules
├── ProjectC
└── ...
```

You will need to modify the **root project** `settings.gradle` to set up your project hierarchy correctly. The important thing is that you will need both `:IHMCOpenRoboticsSoftware` as well as the various `:IHMCOpenRoboticsSoftware:<Sub Project>` items in the Gradle hierarchy, even though IHMCOpenRoboticsSoftware doesn't contain any of its own Java source. This can be tedious to set up by hand, so we usually write our settings.gradle to do some naive dynamic generation of dependencies.

An example `settings.gradle` similar to what our developers use could look like the following:

```gradle
rootProject.name = '_Foo' // Your root project name goes here

def codeDir = rootProject.projectDir

def isGradleProjectFilter = new FilenameFilter() {
    @Override
    boolean accept(File dir, String name) {
        File f = new File(dir, name).getCanonicalFile();
        return f.isDirectory() && new File(f, "build.gradle").exists();
    }
}

codeDir.list(isGradleProjectFilter).each { mainProject ->
    include "${mainProject}"

    project(":${mainProject}").projectDir.list(isGradleProjectFilter).each { subProject ->
        include "${mainProject}:${subProject}"
    }
}
```

If this is set up correctly, you can either apply the `ihmc-build` plugin from the [Plugin portal](https://plugins.gradle.org/plugin/us.ihmc.gradle.ihmc-build) and use the dependency resolver methods exposed by the build extensions, or you can manually identify dependencies on projects using the normal Gradle syntax for project dependencies. A sample build.gradle dependency block:

```gradle
dependencies {
  compile project(':IHMCOpenRoboticsSoftware:IHMCJavaToolkit') // normal Gradle way of doing things
}

/* OR */

dependencies {
  compile ihmc.getProjectDependency(':IHMCJavaToolkit') // ihmc-build way of doing things
}
```
