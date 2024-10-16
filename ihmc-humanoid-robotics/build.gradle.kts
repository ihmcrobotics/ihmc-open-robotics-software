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
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-commons-controls:source")
   api("us.ihmc:ihmc-mecano-tools:source")
}

testDependencies {
   api("us.ihmc:ihmc-communication-test:source")
}
