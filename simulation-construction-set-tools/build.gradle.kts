plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
   id("us.ihmc.scs") version "0.4"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid-frame:0.16.1")
   api("us.ihmc:euclid-frame-shape:0.16.1")
   api("us.ihmc:euclid-shape:0.16.1")
   api("us.ihmc:ihmc-yovariables:0.9.8")
   api("us.ihmc:simulation-construction-set:0.21.5")
   api("us.ihmc:ihmc-parameter-optimization:source")
   api("us.ihmc:ihmc-java-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")

   api("us.ihmc:simulation-construction-set-test:0.21.5")
}
