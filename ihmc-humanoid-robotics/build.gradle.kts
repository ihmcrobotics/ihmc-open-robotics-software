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
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-robotics-tools:0.15.7")
}

testDependencies {
   api("us.ihmc:ihmc-communication-test:source")
}
