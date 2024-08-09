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
   api("us.ihmc:ihmc-humanoid-robotics:source")

   api("us.ihmc:robot-environment-awareness:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robot-data-logger:0.29.1")
   api("us.ihmc:ihmc-path-planning:source")

   api("us.ihmc:ihmc-path-planning-data-sets:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")

   api("us.ihmc:ihmc-quadruped-robotics:source")

   api("us.ihmc:ihmc-whole-body-controller:source")

   api("us.ihmc:ihmc-state-estimation:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
   api("us.ihmc:ihmc-system-identification:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
   api("us.ihmc:ihmc-communication-test:source")

   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("com.google.caliper:caliper:1.0-beta-2")

   api("us.ihmc:simulation-construction-set-tools-test:source")

}
