plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")

   javaDirectory("crocoddyl-wrapper", "generated-java")

   configureDependencyResolution()
   configurePublications()

}

mainDependencies {
   api("gov.nist.math:jama:1.0.3")
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-core:0.39")

   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api(ihmc.sourceSetProject("crocoddyl-wrapper"))
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api(ihmc.sourceSetProject("crocoddyl-wrapper"))
}

