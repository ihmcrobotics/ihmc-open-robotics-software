plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-realtime:1.6.0")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
   api("us.ihmc:ihmc-ros2-library:0.24.4")
   api("org.boofcv:boofcv-geo:0.36")
   api("commons-net:commons-net:3.6")
   api("org.lz4:lz4-java:1.8.0")

   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-whole-body-control-core:source")
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-ros2-library-test:0.24.4")
}
