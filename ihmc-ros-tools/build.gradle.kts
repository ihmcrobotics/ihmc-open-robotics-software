plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   configurePublications()
}

mainDependencies {
   api("io.netty:netty:3.5.8.Final")
   api("com.google.guava:guava:18.0")
   api("org.apache.commons:com.springsource.org.apache.commons.io:1.4.0")
   api("org.boofcv:boofcv-geo:0.36")
   api("org.reflections:reflections:0.9.10")

   api("org.apache.logging.log4j:log4j-1.2-api:2.17.0") // required for rosjava to log stuff
   api("org.ros.rosjava_core:rosjava:0.2.1") {
      exclude(group = "junit", module = "junit")
      exclude(group = "org.ros.rosjava_messages", module = "rosjava_test_msgs")
      exclude(group = "org.ros.rosjava_messages", module = "test_rosmaster")
   }
   api("org.ros.rosjava_bootstrap:message_generation:0.3.3")
   api("org.ros.rosjava_messages:std_msgs:0.5.10")
   api("org.ros.rosjava_messages:std_srvs:1.11.1")
   api("org.ros.rosjava_messages:people_msgs:1.0.4")
   api("org.ros.rosjava_messages:sensor_msgs:1.11.7")
   api("org.ros.rosjava_messages:dynamic_reconfigure:1.5.38")
   api("org.ros.rosjava_messages:multisense_ros:3.4.2")
   api("org.ros.rosjava_messages:rosgraph_msgs:1.11.1")
   api("org.ros.rosjava_messages:geometry_msgs:1.11.7")
   api("org.ros.rosjava_messages:trajectory_msgs:1.11.7")
   api("org.ros.rosjava_messages:nav_msgs:1.11.7")
   api("org.ros.rosjava_messages:tf2_msgs:0.5.9")
   api("org.ros.rosjava_messages:tf:1.10.8")

   api("us.ihmc:euclid:0.19.0")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-java-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("org.ros.rosjava_messages:rosjava_test_msgs:0.2.1")
   api("org.ros.rosjava_messages:test_rosmaster:1.11.10")
}
