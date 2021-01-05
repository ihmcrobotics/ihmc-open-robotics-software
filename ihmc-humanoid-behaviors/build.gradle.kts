plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.thoughtworks.xstream:xstream:1.4.7")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("org.georegression:georegression:0.22")
   api("org.boofcv:boofcv-geo:0.36")
   api("org.boofcv:boofcv-ip:0.36")
   api("org.boofcv:boofcv-io:0.36")
   api("org.boofcv:boofcv-recognition:0.36")
   api("org.postgresql:postgresql:42.2.5")

   api("us.ihmc:ihmc-native-library-loader:1.3.1")
   api("us.ihmc:euclid:0.16.1")
   api("us.ihmc:ihmc-yovariables:0.9.8")
   api("us.ihmc:simulation-construction-set:0.21.5")
   api("us.ihmc:ihmc-graphics-description:0.19.3")
   api("us.ihmc:ihmc-manipulation-planning:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-robot-data-logger:0.20.5")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-perception:source")
   api("us.ihmc:ihmc-footstep-planning:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
