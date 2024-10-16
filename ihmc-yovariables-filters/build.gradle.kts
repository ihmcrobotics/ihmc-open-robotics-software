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
   api("us.ihmc:ihmc-yovariables:0.12.2")
   api("org.ejml:ejml-ddense:0.39")
   api("us.ihmc:ihmc-commons-utils:source")
}

testDependencies {
   api("us.ihmc:ihmc-yovariables-test:0.12.2")
   api("org.apache.commons:commons-math3:3.3")
}
