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
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-core:0.39")
   api("jakarta.xml.bind:jakarta.xml.bind-api:2.3.2")
   api("org.glassfish.jaxb:jaxb-runtime:2.3.2")

   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ekf:0.7.7")
   api("us.ihmc:ihmc-lord-microstrain-drivers:17-0.0.7")
}

testDependencies {
   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
