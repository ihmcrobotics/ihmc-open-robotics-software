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
   api("us.ihmc:ihmc-convex-optimization:0.17.22")
   api("us.ihmc:ihmc-robotics-tools:0.15.0")
   api("us.ihmc:euclid-frame-shape:0.22.2")
   api("us.ihmc:mecano-yovariables-filters:17-0.19.0")
   api("us.ihmc:scs2-definition:17-0.28.1")
}

testDependencies {
   api("us.ihmc:ihmc-convex-optimization-test:0.17.22")
   api("us.ihmc:simulation-construction-set:0.25.2")
   api("us.ihmc:ihmc-commons-testing:0.34.0")
}
