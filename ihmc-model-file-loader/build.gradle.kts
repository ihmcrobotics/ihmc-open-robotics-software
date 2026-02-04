plugins {
   id("us.ihmc.ihmc-build")
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

   api("us.ihmc:ihmc-robot-description:0.25.4")
   api("us.ihmc:ihmc-javafx-toolkit:17-0.23.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
