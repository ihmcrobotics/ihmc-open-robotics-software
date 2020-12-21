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
   api("javax.vecmath:vecmath:1.5.2")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.georegression:georegression:0.22")
   api("org.boofcv:boofcv-geo:0.36")

   api("us.ihmc:ihmc-graphics-description:0.19.3")
   api("us.ihmc:simulation-construction-set:0.21.5")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
}

testDependencies {
   api("us.ihmc:ihmc-communication-test:source")
}
