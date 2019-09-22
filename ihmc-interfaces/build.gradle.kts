import us.ihmc.ros2.rosidl.RosInterfaceGenerator

buildscript {
   repositories {
      maven { url = uri("https://plugins.gradle.org/m2/") }
      mavenCentral()
      maven { url = uri("http://dl.bintray.com/ihmcrobotics/maven-release") }
      maven { url = uri("http://dl.bintray.com/ihmcrobotics/maven-vendor") }
      mavenLocal()
      jcenter()
   }
   dependencies {
      classpath("us.ihmc:ros2-msg-to-pubsub-generator:0.13.4")
   }
}

plugins {
   id("us.ihmc.ihmc-build") version "0.19.5"
   id("us.ihmc.ihmc-ci") version "4.25"
   id("us.ihmc.ihmc-cd") version "0.1"
   id("us.ihmc.log-tools") version "0.3.1"
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
   api("us.ihmc:euclid:0.12.1")
   api("us.ihmc:euclid-geometry:0.12.1")
   api("us.ihmc:ihmc-pub-sub:0.10.2")
   api("us.ihmc:ros2-common-interfaces:0.13.4")
   api("us.ihmc:ihmc-commons:0.26.6")
}

testDependencies {
   api("us.ihmc:ihmc-ros2-library:0.13.4")
}

generatorDependencies {
   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:ros2-msg-to-pubsub-generator:0.13.4")
}

val generator = RosInterfaceGenerator()

tasks.create("generateMessages") {
   doFirst {
      delete("src/main/generated-idl")
      delete("src/main/generated-java")
      delete("src/main/messages/ros1/controller_msgs/msg")
      delete("build/tmp/generateMessages")

      var foundDependency = false

      copy {
         for (file in configurations.compile.get().files)
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
         throw GradleException("Could not find ros2-common-interfaces in configurations.compile!")
      }

      generator.addPackageRootToIDLGenerator(file("build/tmp/generateMessages/ros2-common-interfaces/rcl_interfaces").toPath())
      generator.addPackageRootToIDLGenerator(file("build/tmp/generateMessages/ros2-common-interfaces/common_interfaces").toPath())
      generator.addPackageRootToIDLGenerator(file("src/main/messages/ihmc_interfaces").toPath())
      generator.addPackageRootToROS1Generator(file("src/main/messages/ihmc_interfaces").toPath())

      generator.addCustomIDLFiles(file("build/tmp/generateMessages/ros2-common-interfaces/").toPath())

      generator.generate(file("build/tmp/generateMessages/generated-idl").toPath(),
                         file("build/tmp/generateMessages/generated-ros1").toPath(),
                         file("build/tmp/generateMessages/generated-java").toPath())

      copy {
         from("build/tmp/generateMessages/generated-idl/controller_msgs")
         into("src/main/generated-idl/controller_msgs")
      }

      copy {
         from("build/tmp/generateMessages/generated-java/controller_msgs")
         into("src/main/generated-java/controller_msgs")
      }

      copy {
         from("build/tmp/generateMessages/generated-ros1/controller_msgs")
         into("src/main/messages/ros1/controller_msgs")
      }

      RosInterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/generated-idl").toPath())
      RosInterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/generated-java").toPath())
      RosInterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/messages/ros1").toPath())
   }
}