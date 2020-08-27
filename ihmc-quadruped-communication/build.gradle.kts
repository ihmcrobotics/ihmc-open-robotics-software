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
   api("us.ihmc.thirdparty.jinput:jinput:200128")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid-geometry:0.15.1")
   api("us.ihmc:ihmc-yovariables:0.9.3")
   api("us.ihmc:ihmc-quadruped-basics:source")
   api("us.ihmc:ihmc-quadruped-planning:source")
   api("us.ihmc:ihmc-quadruped-footstep-planning:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
}

testDependencies {
   api("us.ihmc:ihmc-quadruped-footstep-planning-visualizers:source")
}
