plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("net.sf.trove4j:trove4j:3.0.3")
   api("us.ihmc:ihmc-realtime:1.5.0")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
   api("org.boofcv:boofcv-geo:0.36")
   api("com.google.guava:guava:18.0")
   api("org.reflections:reflections:0.9.10")
   api("commons-net:commons-net:3.6")
   api("net.jpountz.lz4:lz4:1.3.0")

   api("us.ihmc:euclid:0.17.2")
   api("us.ihmc:euclid-geometry:0.17.2")
   api("us.ihmc:ihmc-ros2-library:0.20.5")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-interfaces:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-ros2-library-test:0.20.5")
}
