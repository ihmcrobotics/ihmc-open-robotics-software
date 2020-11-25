plugins {
   id("us.ihmc.ihmc-build") version "0.22.0"
   id("us.ihmc.ihmc-ci") version "7.0"
   id("us.ihmc.ihmc-cd") version "1.16"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-quadruped-robotics:source")
   api("us.ihmc:ihmc-robot-models-visualizers:source")
   api("us.ihmc:ihmc-quadruped-footstep-planning-visualizers:source")
}

testDependencies {
}
