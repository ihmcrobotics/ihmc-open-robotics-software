plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

apply(from = "../gradle/java-compile-encoding.gradle.kts")

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:simulation-construction-set:0.25.4") // SCS1
   api("us.ihmc:ihmc-whole-body-controller:source")
}

testDependencies {
   api("us.ihmc:ihmc-communication-test:source")
}
