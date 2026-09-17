plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   configurePublications()
}

mainDependencies {
   api("commons-collections:commons-collections:3.2.1")
   api("org.jgrapht:jgrapht-core:0.9.0")
   api("org.jgrapht:jgrapht-ext:0.9.0")
   api("com.github.wendykierp:JTransforms:3.1")

   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robot-models:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
