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
   api("us.ihmc:scs2-definition:17-0.27.3")
   api("us.ihmc:ihmc-graphics-description:0.25.1")
   api("us.ihmc:mecano-yovariables:17-0.18.1")
   api("us.ihmc:ihmc-yovariables-filters:source")
   api("jgraph:jgraph:5.13.0.0")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:ihmc-matrix-library:0.19.0")
   api("us.ihmc:euclid-test:0.21.0")
   api("org.apache.commons:commons-math3:3.3")
}
