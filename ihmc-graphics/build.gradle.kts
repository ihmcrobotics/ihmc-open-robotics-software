plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-sensor-processing:source")
}

testDependencies {

}

javafxDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.9")
   api("us.ihmc:simulation-construction-set-tools:source")
}

javafxTestDependencies {
   api(ihmc.sourceSetProject("javafx"))
}

jmonkeyengineDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set-tools:source")

   var javaFXVersion = "17.0.9"
   api(ihmc.javaFXModule("graphics", javaFXVersion)) // JFX Color
}

jmonkeyengineTestDependencies {
   api(ihmc.sourceSetProject("jmonkeyengine"))
}
