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
   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:euclid-update:source")
   api("us.ihmc:euclid-update-frame:source")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:mecano:17-0.18.1")
   api("jgraph:jgraph:5.13.0.0")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:ihmc-matrix-library:0.19.0")
   api("us.ihmc:euclid-test:0.21.0")
   api("us.ihmc:euclid-update:source")
   api("us.ihmc:euclid-update-frame:source")
   api("org.apache.commons:commons-math3:3.3")
}
