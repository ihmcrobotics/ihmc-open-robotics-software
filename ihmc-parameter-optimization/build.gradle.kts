plugins {
   id("us.ihmc.ihmc-build") version "0.22.0"
   id("us.ihmc.ihmc-ci") version "7.1"
   id("us.ihmc.ihmc-cd") version "1.16"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-yovariables:0.9.6")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
