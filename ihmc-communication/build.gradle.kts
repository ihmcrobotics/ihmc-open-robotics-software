plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-realtime:1.7.1")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
   api("us.ihmc:jros2:source")
   api("us.ihmc:ros2-library:1.2.5")  // Keep temporarily for helper utilities
   api("us.ihmc:ihmc-pub-sub-serializers-extra:1.2.5")
   api("commons-net:commons-net:3.6")
   api("org.lz4:lz4-java:1.8.0")
   api("com.github.crykn:kryonet:2.22.7") // from jitpack

   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-interfaces-jros2:source")
   api("us.ihmc:ihmc-java-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-graphics-libgdx:source")
   api("us.ihmc:ros2-library-test:1.2.5")
}
