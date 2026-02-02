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
   api("com.thoughtworks.xstream:xstream:1.4.19")
   api("com.github.wendykierp:JTransforms:3.1")

   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
