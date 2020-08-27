plugins {
   id("us.ihmc.ihmc-build") version "0.21.0"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
      
   configurePublications()
}

mainDependencies {
   api("gov.nist.math:jama:1.0.3")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("com.google.guava:guava:18.0")
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-core:0.39")
   api("net.java.dev.jna:jna:4.1.0")
   api("net.sf.trove4j:trove4j:3.0.3")

   api("us.ihmc:ihmc-realtime:1.3.0")
   api("us.ihmc:ihmc-native-library-loader:1.2.1")
   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:euclid-geometry:0.15.1")
   api("us.ihmc:ihmc-yovariables:0.9.3")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-trajectory-optimization:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-convex-optimization:0.17.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
}

testDependencies {

   api("us.ihmc:ihmc-commons-testing:0.30.3")
   api("us.ihmc:simulation-construction-set:0.20.4")
   api("us.ihmc:simulation-construction-set-test:0.20.4")
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-convex-optimization-test:0.17.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:simulation-construction-set:0.20.4")
}
