plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-avatar-interfaces:source")
   api("us.ihmc:ihmc-user-interface:1.0.4-20231109")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-avatar-interfaces-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-commons-testing:0.32.0")
}
