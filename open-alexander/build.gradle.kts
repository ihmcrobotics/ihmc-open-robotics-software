plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-alex-sdk:0.3.7")
   api("us.ihmc:ihmc-avatar-interfaces-visualizers:source")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-manipulation-planning:source")
   api("us.ihmc:ihmc-footstep-planning-visualizers:source")
   api("us.ihmc:ihmc-high-level-behaviors:source")
}

testDependencies {
   api("us.ihmc:ihmc-avatar-interfaces-test:source")
   api("us.ihmc:ihmc-sensor-processing-test:source")
   api("us.ihmc:ihmc-simulation-toolkit-test:source")
}