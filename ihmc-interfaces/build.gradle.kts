buildscript {
   repositories {
      maven { url = uri("https://plugins.gradle.org/m2/") }
      mavenCentral()
      mavenLocal()
   }
   dependencies {
      classpath("us.ihmc:ros2-msg-to-pubsub-generator:0.24.1")
      classpath("us.ihmc:log-tools:0.6.3") // removes vulnerable log4j versions from plugin classpath; can be removed later
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   resourceDirectory("main", "messages")
   resourceDirectory("main", "generated-idl")
   javaDirectory("main", "generated-java")
   resourceDirectory("generator", "docker")
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:ihmc-pub-sub:0.19.1")
   api("us.ihmc:ros2-common-interfaces:0.24.1") {
      exclude(group = "org.junit.jupiter", module = "junit-jupiter-api")
      exclude(group = "org.junit.jupiter", module = "junit-jupiter-engine")
      exclude(group = "org.junit.platform", module = "junit-platform-commons")
      exclude(group = "org.junit.platform", module = "junit-platform-launcher")
   }
   api("us.ihmc:ihmc-commons:0.32.0")
}

testDependencies {
   api("us.ihmc:ihmc-ros2-library:0.24.1")
}

generatorDependencies {
   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:ros2-msg-to-pubsub-generator:0.24.1")
}

val generator = us.ihmc.ros2.rosidl.ROS2InterfaceGenerator()
val msg_packages = listOf("ihmc_common_msgs",
                          "mission_control_msgs",
                          "controller_msgs",
                          "toolbox_msgs",
                          "quadruped_msgs",
                          "perception_msgs",
                          "behavior_msgs",
                          "exoskeleton_msgs",
                          "atlas_msgs",
                          "test_msgs")

tasks.create("generateMessages") {
   doFirst {
      delete("src/main/generated-idl")
      delete("src/main/generated-java")
      delete("build/tmp/generateMessages")

      for (packag in msg_packages) {
         delete("src/main/messages/ros1/" + packag + "/msg")
      }

      var foundDependency = false

      copy {
         for (file in configurations.runtimeClasspath.get().files)
         {
            if (file.name.contains("ros2-common-interfaces"))
            {
               from(zipTree(file))
               foundDependency = true
            }
            into(file("build/tmp/generateMessages/ros2-common-interfaces"))
         }
      }

      if (!foundDependency)
      {
         throw GradleException("Could not find ros2-common-interfaces in configurations.runtimeClasspath!")
      }

      generator.addPackageRootToIDLGenerator(file("build/tmp/generateMessages/ros2-common-interfaces/rcl_interfaces").toPath())
      generator.addPackageRootToIDLGenerator(file("build/tmp/generateMessages/ros2-common-interfaces/common_interfaces").toPath())
      generator.addPackageRootToIDLGenerator(file("src/main/messages/ihmc_interfaces").toPath())
      generator.addPackageRootToROS1Generator(file("src/main/messages/ihmc_interfaces").toPath())

      generator.addCustomIDLFiles(file("build/tmp/generateMessages/ros2-common-interfaces/").toPath())

      generator.generate(file("build/tmp/generateMessages/generated-idl").toPath(),
                         file("build/tmp/generateMessages/generated-ros1").toPath(),
                         file("build/tmp/generateMessages/generated-java").toPath())

      for (packag in msg_packages) {
         copy {
            from("build/tmp/generateMessages/generated-idl/$packag")
            into("src/main/generated-idl/$packag")
         }

         copy {
            from("build/tmp/generateMessages/generated-java/$packag")
            into("src/main/generated-java/$packag")
         }

         copy {
            from("build/tmp/generateMessages/generated-ros1/$packag")
            into("src/main/messages/ros1/$packag")
         }
      }

      us.ihmc.ros2.rosidl.ROS2InterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/generated-idl").toPath())
      us.ihmc.ros2.rosidl.ROS2InterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/generated-java").toPath())
      us.ihmc.ros2.rosidl.ROS2InterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/messages/ros1").toPath())
   }
}
