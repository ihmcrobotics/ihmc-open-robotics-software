plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.20"
   id("us.ihmc.log-tools-plugin") version "0.6.2"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("io.javaslang:javaslang:2.0.3")

   api("us.ihmc:euclid:0.16.2")
   api("us.ihmc:ihmc-javafx-toolkit:0.19.3")
   api("us.ihmc:ihmc-robot-description:0.21.2")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.19.7")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-java-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
