plugins {
   id("us.ihmc.ihmc-build") version "0.21.0"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.15.1")
   api("org.ddogleg:ddogleg:0.18")

   api("us.ihmc:ihmc-yovariables:0.9.3")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
   api("us.ihmc:simulation-construction-set:0.20.4")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
