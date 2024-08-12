plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-state-estimation:source")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:simulation-construction-set-tools:source")
}

testDependencies {
   api("us.ihmc:ihmc-humanoid-robotics-test:source")
}
