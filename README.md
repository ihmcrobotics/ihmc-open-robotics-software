# IHMC Open Robotics Software

### Tested Platforms
We test all of our software on OS X 10.12 Sierra, Windows 7/8/10, and Ubuntu 14.04 and 16.04 LTS, Desktop and Server. It is likely to work on other platforms but
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

   /*  Artifactory instance hosted at IHMC for some legacy vendored
    *  dependencies without live source access that can't be uploaded
    *  to Bintray.
   */
   maven {
   		url "https://artifactory.ihmc.us/artifactory/thirdparty/"
   }

   /* You will also need to add either jcenter() or mavenCentral() or both, depending on your preference */
}
```

### Developing with *IHMC Open Robotics Software* from source

#### Requirements
*IHMC Open Robotics Software* uses the [Gradle](https://gradle.org) build system, and requires JDK 8 with JavaFX. We also strongly suggest an IDE, either Eclipse Mars.1
or IntelliJ IDEA 15+ (Ultimate or Community is fine). Currently, we require **Gradle 4.1+**. We provide a versioned [Gradle wrapper](https://docs.gradle.org/current/userguide/gradle_wrapper.html)
for getting started quickly. The Gradle wrapper will always reflect the minimum version of Gradle required to build the software; if we adopt features only present
in newer versions of Gradle as they are release we will update the wrapper. You can also install Gradle system-wide:

* *OS X*: The latest Gradle is available via [Homebrew](https://github.com/homebrew/homebrew)
* *Ubuntu*: The version pre-packaged with Ubuntu is woefully out of date. You will need to add [this PPA](https://launchpad.net/~cwchien/+archive/ubuntu/gradle)
* *Windows*: Consult the official Gradle documentation installation instructions.

#### IDE Support
Our Gradle models are tested in IntelliJ IDEA 15+ (both Community and Ultimate) with the built-in Gradle integration, and Eclipse Mars.1 or higher with the Buildship
plugin. The Buildship plugin is bundled with the Eclipse IDE for Java Developers as of Mars.1 (but *not* Java EE Developers), or it can be downloaded from the Eclipse
Marketplace in other versions of Eclipse.

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
$ ./gradlew publishAllToMavenLocal
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
  compile group: 'us.ihmc', name: 'simulation-construction-set', version: 'LOCAL', changing: true
}
```  

#### Depending directly on the source
For *IHMC Open Robotics Software* and [ihmc-build](https://github.com/ihmcrobotics/ihmc-build) to work correctly when depending directly on the source, your
project hierarchy needs to take a particular form. In your system home folder, create a directory called `ihmc-workspace`.Inside that directory, this repo and
create the files `build.gradle`, `settings.gradle`, `gradle.properties`. This is also the directory where you'll put any of your own projects that need to depend
on IHMC source code. Your directory structure should look something like:

```
ihmc-workspace
├── build.gradle
├── settings.gradle
├── gradle.properties
├── MyProjectA
│   └── build.gradle
│   └── gradle.properties
├── MyProjectB
├── ihmc-open-robotics-software
│   └── acsell
│   └── atlas
│   └── common-walking-control-modules
├── MyProjectC
└── ...
```

Copy the following text into the `build.gradle` in your `ihmc-workspace` directory:

```gradle
buildscript {
   repositories {
      maven { url "https://plugins.gradle.org/m2/" }
      mavenLocal()
      jcenter()
   }
   dependencies {
      classpath "gradle.plugin.us.ihmc:ihmc-build:0.9.5"
   }
}
apply plugin: "us.ihmc.ihmc-build"
```

Copy the following text into the `settings.gradle` in your `ihmc-workspace` directory:

```gradle
buildscript {
   repositories {
      maven { url "https://plugins.gradle.org/m2/" }
      mavenLocal()
   }
   dependencies {
      classpath "gradle.plugin.us.ihmc:ihmc-build:0.9.5"
   }
}

import us.ihmc.build.IHMCSettingsConfigurator

def ihmcSettingsConfigurator = new IHMCSettingsConfigurator(settings, logger, ext)
ihmcSettingsConfigurator.configureProjectName(hyphenatedName) // rootProject.name = hyphenatedName
ihmcSettingsConfigurator.configureAsGroupOfProjects() // isProjectGroup = true

if (Boolean.valueOf(includeBuildsFromWorkspace))
{
   ihmcSettingsConfigurator.findAndIncludeCompositeBuilds() // Search workspace and `includeBuild` matches
}
```

Copy the following text into the `gradle.properties` in your `ihmc-workspace` directory:

```gradle
isProjectGroup = true
hyphenatedName = ihmc-robotics
pascalCasedName = IHMCRobotics
publishMode = SNAPSHOT
groupDependencyVersion = SNAPSHOT-LATEST
depthFromWorkspaceDirectory = 0
includeBuildsFromWorkspace = true
excludeFromCompositeBuild = false
artifactoryUsername = unset_username
artifactoryPassword = unset_password
bintray_user = unset_user
bintray_key = unset_key
```

If this is set up correctly, you can either apply the `ihmc-build` plugin from the [Plugin portal](https://plugins.gradle.org/plugin/us.ihmc.gradle.ihmc-build)
and use the dependency resolver methods exposed by the build extensions, or you can manually identify dependencies on projects using the normal Gradle syntax for
project dependencies. A sample build.gradle dependency block:

```gradle
dependencies {
  compile project(':ihmc-open-robotics-software:ihmc-java-toolkit') // normal Gradle way of doing things
}

/* OR */

mainDependencies {
  compile compile group: "us.ihmc", name: "ihmc-java-toolkit", version: groupDependencyVersion // ihmc-build way of doing things
}
```
