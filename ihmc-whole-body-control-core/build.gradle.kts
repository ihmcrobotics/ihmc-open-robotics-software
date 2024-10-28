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
   api("us.ihmc:ihmc-robotics-tools:source")
   api("us.ihmc:euclid-frame-shape:0.22.0")
   api("us.ihmc:ihmc-yovariables-filters:0.13.1")
   api("us.ihmc:mecano-yovariables-filters:17-0.18.1")
   api("us.ihmc:ihmc-graphics-description-update:source")
}

testDependencies {
   api("us.ihmc:ihmc-convex-optimization-test:0.17.21")
   api("us.ihmc:simulation-construction-set:0.25.1")
   api("us.ihmc:ihmc-commons-testing:0.33.0")
}
