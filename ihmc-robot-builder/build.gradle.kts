plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("io.javaslang:javaslang:2.0.3")

   api("us.ihmc:euclid:0.15.2")
   api("us.ihmc:ihmc-javafx-toolkit:0.19.2")
   api("us.ihmc:ihmc-robot-description:0.20.2")
   api("us.ihmc:ihmc-graphics-description:0.19.2")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.19.2")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-java-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
