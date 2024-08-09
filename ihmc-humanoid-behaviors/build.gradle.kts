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
   api("org.postgresql:postgresql:42.2.5")

   api("us.ihmc:ihmc-manipulation-planning:source")
   api("us.ihmc:ihmc-footstep-planning:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
