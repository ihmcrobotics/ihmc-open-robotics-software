plugins {
   id("us.ihmc.ihmc-build") version "0.21.0"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

// compileJava {
//    options.compilerArgs += ["--add-modules", "java.xml.bind"]
// }

mainDependencies {
   api("org.apache.commons:commons-lang3:3.8.1")
   api("jakarta.xml.bind:jakarta.xml.bind-api:2.3.2")
   api("org.glassfish.jaxb:jaxb-runtime:2.3.2")

   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:ihmc-robot-description:0.20.1")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
   api("us.ihmc:ihmc-javafx-toolkit:0.19.1")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api("us.ihmc:simulation-construction-set-test:0.20.4")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
