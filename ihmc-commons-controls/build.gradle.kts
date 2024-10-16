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
   api("us.ihmc:mecano:17-0.18.1")
   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:ihmc-commons-utils:source")
   api("net.sf.trove4j:trove4j:3.0.3")
   api("com.google.guava:guava:18.0")
}

testDependencies {
}
