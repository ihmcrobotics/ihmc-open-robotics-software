plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
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
   api("org.georegression:georegression:0.22")
   api("org.ddogleg:ddogleg:0.18")
   api("org.apache.commons:commons-math3:3.6.1")
   api("com.thoughtworks.xstream:xstream:1.4.19")
   api("org.jgrapht:jgrapht-core:0.9.0")
   api("org.jgrapht:jgrapht-ext:0.9.0")
   api("com.github.wendykierp:JTransforms:3.1")
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-core:0.39")
   api("jgraph:jgraph:5.13.0.0")
   api("org.boofcv:boofcv-geo:0.36")

   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-perception:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-robot-models:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
