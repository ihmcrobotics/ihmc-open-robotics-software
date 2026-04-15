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
   api("us.ihmc:ihmc-robot-data-logger:0.37.3")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-math-linear-dynamic-systems:0.15.7")
   api("us.ihmc:scs2-examples:17-0.32.1")

   api("us.ihmc:ihmc-simulation-toolkit:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
   api("us.ihmc:ihmc-communication-test:source")

   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("com.google.caliper:caliper:1.0-beta-2")

   api("us.ihmc:simulation-construction-set-tools-test:source")

}
