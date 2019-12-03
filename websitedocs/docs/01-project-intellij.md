---
title: Create your First GradleProject with a build.gradle File
sidebar_label: Create your First GradleProject
---


Once you have the prerequisites installed, we'll get started by setting up a project that depends on the IHMC `Valkyrie` package, which contains all of the code needed to simulate the [NASA Johnson Space Center Valkyrie](http://nasa-jsc-robotics.github.io/valkyrie/) Humanoid robot in IHMC's Simulation Construction Set, as well as the code needed to run our software on the real Valkyrie robot. We'll do this by setting up a *Gradle project* which references some IHMC Maven artifacts.

<br/>
### Create the Project Directory Structure and the `build.gradle` File

Create a directory called `GradleProject` and create the folder where your source code lives; the Java convention enforced by Gradle is to have your source code live at `src/main/java`. You should have a layout on disk like this:

    GradleProject
    ├── src
    │   └── main
    │       └── java
    └── build.gradle

<br/>
### Create the `build.gradle` File

In your `GradleProject` folder create a file named `build.gradle` with the following contents:

```java

apply plugin: 'java'

repositories {
   maven {
      url  "https://dl.bintray.com/ihmcrobotics/maven-release" // IHMC Code releases
   }

   maven {
      url  "https://dl.bintray.com/ihmcrobotics/maven-vendor" // Third-party libraries that we have vendored for various reasons
   }

   /*  
    *  Maven repos hosted at IHMC for some legacy vendored
    *  dependencies we have not been able to vendor on Bintray yet.
    *  This will be going away eventually.
    */
   maven {
        url "https://bengal.ihmc.us/nexus/content/repositories/thirdparty/"
   }

   jcenter() // One of the central Maven repos. You can also use mavenCentral() instead or in addition to.
}

dependencies {
   compile 'us.ihmc:Valkyrie:{{OpenSourceVersion}}' // <- Group: us.ihmc, Artifact: Valkyrie, Version: {{OpenSourceVersion}}
}

```





