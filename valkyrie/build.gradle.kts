plugins {
   id("us.ihmc.ihmc-build") version "0.19.6"
   id("us.ihmc.ihmc-ci") version "5.0"
   id("us.ihmc.ihmc-cd") version "1.7"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools") version "0.3.1"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.martiansoftware:jsap:2.1")
   api("org.yaml:snakeyaml:1.17") //1.11
   api("org.ejml:core:0.30")
   api("org.ejml:simple:0.30")
   api("org.ejml:dense64:0.30")
   api("com.esotericsoftware.minlog:minlog:1.2")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("us.ihmc.thirdparty.jinput:jinput:190214")
   api("org.ros.rosjava_bootstrap:message_generation:0.2.1")
   api("org.ros.rosjava_messages:std_msgs:0.5.9")

   api("us.ihmc:euclid:0.12.1")
   api("us.ihmc:ihmc-yovariables:0.3.11")
   api("us.ihmc:ihmc-realtime:1.2.6")
   api("us.ihmc:IHMCRosControl:0.5.0") {
      setChanging(true)
   }
   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.12.8")
   api("us.ihmc:simulation-construction-set:0.12.15")
   api("us.ihmc:ihmc-graphics-description:0.12.12")
   api("us.ihmc:ihmc-robot-description:0.12.7")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-system-identification:source")
   api("us.ihmc:ihmc-state-estimation:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-robot-data-logger:source")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-perception:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
   api("us.ihmc:ihmc-footstep-planning-visualizers:source")
   api("us.ihmc:ihmc-avatar-interfaces-behavior-fx-ui:source")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.26.6")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-avatar-interfaces-test:source")
}

ihmc.jarWithLibFolder()
tasks.getByPath("installDist").dependsOn("compositeJar")

app.entrypoint("IHMCValkyrieJoystickApplication", "us.ihmc.valkyrie.joystick.ValkyrieJoystickBasedSteppingApplication")
app.entrypoint("valkyrie-network-processor", "us.ihmc.valkyrie.ValkyrieNetworkProcessor")

tasks.create("deployOCUApplications") {
   dependsOn("installDist")

   doLast {
      val appFolder = File(System.getProperty("user.home"), "ihmc_apps/valkyrie")
      appFolder.delete()
      appFolder.mkdirs()
      copy {
         from("build/install/valkyrie")
         into(appFolder)
      }
      println("-------------------------------------------------------------------------")
      println("------- Deployed files to: " + appFolder.path + " -------")
      println("-------------------------------------------------------------------------")
   }
}

tasks.create("deployLocal") {
   dependsOn("installDist")

   doLast {
      val libFolder = File(System.getProperty("user.home"), "valkyrie/lib")
      libFolder.delete()
      libFolder.mkdirs()

      copy {
         from("build/install/valkyrie/lib")
         into(libFolder)
      }

      copy {
         from("build/libs/valkyrie-$version.jar")
         into(File(System.getProperty("user.home"), "valkyrie"))
      }

      val configurationDir = File(System.getProperty("user.home"), ".ihmc/Configurations")
      configurationDir.delete()
      configurationDir.mkdirs()

      copy {
         from("saved-configurations/defaultREAModuleConfiguration.txt")
         into(configurationDir)
      }
   }
}

val directory = "/home/val/valkyrie"

tasks.create("deploy") {
   dependsOn("installDist")

   doLast {
      remote.session("link02", "val") // control
      {
         exec("mkdir -p $directory")

         exec("rm -rf $directory/lib")
         put(file("build/install/valkyrie/lib").toString(), "$directory/lib")
         exec("ls -halp $directory/lib")

         put(file("build/libs/valkyrie-$version.jar").toString(), "$directory/ValkyrieController.jar")
         put(file("launchScripts").toString(), directory)
         exec("chmod +x $directory/runNetworkProcessor.sh")
         exec("ls -halp $directory")
      }

      deployNetworkProcessor()
   }
}

tasks.create("deployNetworkProcessor") {
   dependsOn("installDist")

   doLast {
      deployNetworkProcessor()
   }
}

fun deployNetworkProcessor()
{
   remote.session("zelda02", "val") // perception
   {
      exec("mkdir -p $directory")

      exec("rm -rf $directory/bin")
      exec("rm -rf $directory/lib")

      put(file("build/install/valkyrie/bin").toString(), "$directory/bin")
      exec("chmod +x $directory/bin/valkyrie-network-processor")
      put(file("build/install/valkyrie/lib").toString(), "$directory/lib")
      exec("ls -halp $directory/lib")

      put(file("build/libs/valkyrie-$version.jar").toString(), "$directory/ValkyrieController.jar")
      put(file("launchScripts").toString(), directory)
      exec("chmod +x $directory/runNetworkProcessor.sh")
      exec("ls -halp $directory")

      exec("rm -rf /home/val/.ihmc/Configurations")
      exec("mkdir -p /home/val/.ihmc/Configurations")
      put(file("saved-configurations/defaultREAModuleConfiguration.txt").toString(), ".ihmc/Configurations")
      exec("ls -halp /home/val/.ihmc/Configurations")
   }
}