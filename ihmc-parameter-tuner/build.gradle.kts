plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("org.apache.commons:commons-math3:3.3")
   api("jakarta.xml.bind:jakarta.xml.bind-api:2.3.2")
   api("org.glassfish.jaxb:jaxb-runtime:2.3.2")

   api("us.ihmc:ihmc-yovariables:0.9.8")
   api("us.ihmc:ihmc-javafx-toolkit:0.19.3")
   api("us.ihmc:ihmc-robot-data-logger:0.20.5")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
    api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
