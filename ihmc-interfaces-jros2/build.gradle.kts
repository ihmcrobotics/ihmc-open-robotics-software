import us.ihmc.jros2.generator.jros2GenTask

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
   id("us.ihmc.jros2.generator") version "1.2.2"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:jros2:source")
   api("us.ihmc:euclid-geometry:0.22.5")
   api("us.ihmc:euclid-shape:0.22.5")
   api("us.ihmc:ihmc-commons:0.35.1")
}

tasks.register<jros2GenTask>("generateMessages") {
   description = "Generate IHMC custom ROS 2 interfaces using jros2"
   group = "build"

   packagePaths = listOf(
      projectDir.resolve("messages/ihmc_interfaces/behavior_msgs").absolutePath,
      projectDir.resolve("messages/ihmc_interfaces/controller_msgs").absolutePath,
      projectDir.resolve("messages/ihmc_interfaces/ihmc_common_msgs").absolutePath,
      projectDir.resolve("messages/ihmc_interfaces/perception_msgs").absolutePath,
      projectDir.resolve("messages/ihmc_interfaces/system_monitor_msgs").absolutePath,
      projectDir.resolve("messages/ihmc_interfaces/test_msgs").absolutePath,
      projectDir.resolve("messages/ihmc_interfaces/toolbox_msgs").absolutePath,
      projectDir.resolve("messages/ihmc_interfaces/vision_msgs").absolutePath
   )

   outputDir = sourceSets["main"].java.srcDirs.find { it.name == "java" }!!.absolutePath

   // Map standard geometry_msgs types to custom Euclid wrapper classes
   typeToClass = mapOf(
      "geometry_msgs/Point" to "us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage",
      "geometry_msgs/Vector3" to "us.ihmc.euclid.jros2.messages.EuclidVector3DMessage",
      "geometry_msgs/Quaternion" to "us.ihmc.euclid.jros2.messages.EuclidQuaternionMessage",
      "geometry_msgs/Pose" to "us.ihmc.euclid.jros2.messages.EuclidPose3DMessage",
      "geometry_msgs/Transform" to "us.ihmc.euclid.jros2.messages.EuclidTransformMessage"
   )
}

// Make compileJava depend on message generation
tasks.named("compileJava") {
   dependsOn("generateMessages")
}
