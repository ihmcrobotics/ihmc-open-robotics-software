plugins {
   id("us.ihmc.ihmc-build") version "0.22.0"
   id("us.ihmc.ihmc-ci") version "6.8"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.apache.commons:commons-lang3:3.8.1")
   api("org.ejml:ejml-core:0.39")
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.georegression:georegression:0.22")
   api("org.ejml:ejml-ddense:0.39")
   api("org.boofcv:boofcv-geo:0.36")

   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:ihmc-yovariables:0.9.6")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-interfaces:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-communication-test:source")
}
