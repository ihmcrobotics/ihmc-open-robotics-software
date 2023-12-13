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

   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:euclid-shape:0.21.0")
   api("us.ihmc:jinput:2.0.6-ihmc2")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-quadruped-robotics:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-quadruped-robotics-test:source")
   api("us.ihmc:ihmc-quadruped-planning-test:source")

}
