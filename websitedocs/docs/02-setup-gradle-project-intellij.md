---
title: Import GradleProject
sidebar_label: Import GradleProject
---

<br/> 
### Gradle Plugin Support

IntelliJ IDEA supports Gradle SDK version 1.2 and later, and the Gradle plugin is bundled with IntelliJ IDEA and activated by default.  If the plugin is not activated, enable it on the `Plugins` tab of the `File > Settings` (Windows and Ubuntu) or `IntelliJ IDEA > Preferences`(OSX) menu.

You can find more detailed information about Gradle and IntelliJ on [IntelliJ's website](https://www.jetbrains.com/help/idea/2016.1/getting-started-with-gradle.html). 

<br/>
### Opening/Importing GradleProject

If you have just started IntelliJ IDEA, you will be presented with the Welcome screen.

![welcome screen](/img/quickstart/intellij/welcomeScreen.png)

Choose `Open` from the Welcome screen, or if IntelliJ IDEA is already open choose `File/Open` from the menu bar.

![import build gradle project](/img/quickstart/intellij/open-build-gradle.png)

From the `Open File or Project` dialog traverse the directory structure and select the `GradleProject` folder or the `build.gradle` file and select `OK`.

There are several things to note in the next dialog, which have been marked:

![import build gradle project](/img/quickstart/intellij/import-project-from-gradle.png)

1. We highly recommend **unchecking** the option for using separate modules per source set
2. If you are using a local Gradle distribution, you will need to identify the Gradle Home; typically this is the directory *above* the `bin` directory that contains the `gradle` executable. In this example, using Homebrew for OS X, we identify `/usr/local/opt/gradle/libexec`; if you were to look inside of this directory you would see `/usr/local/opt/gradle/libexec/bin/gradle`. On other operating systems this location may change; for example, in Ubuntu, you may need to traverse several symlinks due to the way apt-get installed packages are configured in order to find the actual Gradle Home.
3. If this is your first time launching IntelliJ IDEA, the "Gradle JVM" option will be empty and will make allusions to Java Home. Another option is to have already added a "Project SDK" to IntelliJ using the Wizard. This can be done **prior to importing the build.gradle** by selecting `Configure -> Project Defaults -> Structure` and using the `New` button to add a JDK installation to IntelliJ IDEA:

![configure project structure](/img/quickstart/intellij/project-structure.png)

![add sdk](/img/quickstart/intellij/add-sdk.png)

Wait few minutes while IntelliJ IDEA imports GradleProject and obtains the dependencies specified in the `build.gradle` file:

![downloading dependencies](/img/quickstart/intellij/import-build-gradle.png)



<!--Alternatively you can chose to create a new project and copy the content of your `build.gradle` script. -->
 <!---->
 <!--Select `File > New Project...` to open the Project Wizard.  -->
 <!--On the left pane select **Gradle**   -->
 <!--On the right side of the panel specify your project SDK(JDK) and make sure that the *Java* checkbox is selected in the **Additional Libraries and Frameworks** area.   -->
 <!--Click `Next`.  -->
 <!---->
<!--On the next page of the wizard specify the following setting:-->

<!--**GroupId** - specify groupId of the new project, which will be added to the build.gradle file.  -->
<!--**ArtifactId** - specify artifactId of the new project.  -->
<!--**Version** - specify version of the new project, which will be added to the build.gradle file.  -->

<!--[IMAGE HERE]-->

<!--Click `Next`.-->

<!--On the next page, specify the Gradle Settings:-->

<!--[IMAGE HERE]-->

<!--Select "Create directories for empty content roots automatically" checkbox.  -->
<!--Select "Use default gradle wrapper(recommended)" radio button.  -->
<!--Make sure that "Gradle JMV" contains the path to your JDK. If this is not the case, you probably need to specify the JAVA_HOME environment variable.   -->
<!--Click `Next`.   -->

<!--On the next page, specify the name and location settings. -->
<!--[IMAGE HERE]-->

<!--At this point IntelliJ IDEA automatically creates a project with a default 'build.gradle' file.-->

<!--Your environment should now look like this: -->
<!--[IMAGE HERE]-->

<!--Double click on build.gradle to open it. -->
<!--Edit it so that it contains the fields that you specified in the build.gradle script you defined earlier. -->
<!--[IMAGE HERE]-->

