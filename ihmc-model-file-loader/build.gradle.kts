plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.24"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
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
   api("jakarta.xml.bind:jakarta.xml.bind-api:2.3.2")
   api("org.glassfish.jaxb:jaxb-runtime:2.3.2")

   api("us.ihmc:euclid:0.20.0")
   api("us.ihmc:ihmc-robot-description:0.21.10")
   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.3")
   api("us.ihmc:scs2-definition:17-0.14.7")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
