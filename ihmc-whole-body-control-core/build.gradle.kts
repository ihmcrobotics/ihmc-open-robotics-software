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
   api("us.ihmc:ihmc-convex-optimization:0.17.21")
   api("us.ihmc:ihmc-commons-controls:source")
   api("us.ihmc:ihmc-commons-utils:source")
   api("us.ihmc:ihmc-yovariables-filters:source")
   api("us.ihmc:mecano-yovariables-filters:source")
}

testDependencies {
   api("us.ihmc:ihmc-convex-optimization-test:0.17.21")
   api("us.ihmc:simulation-construction-set:0.25.1")
   api("us.ihmc:ihmc-commons-testing:0.32.0")
}
