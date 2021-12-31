plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.5"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-avatar-interfaces:source") {
      exclude(group = "org.openjfx")
      exclude(group = "org.jmonkeyengine")
      exclude(group = "org.lwjgl.lwjgl") // exclude lwjgl 2
   }
}

javafxDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-javafx-toolkit:0.19.3")
   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:robot-environment-awareness-visualizers:source")
}

libgdxDependencies {
   api(ihmc.sourceSetProject("main"))
   api("org.lwjgl:lwjgl-opencl:3.2.3")
   api("us.ihmc:ihmc-graphics-libgdx:source")
   api("us.ihmc:ihmc-perception-javacv:source")
   api("us.ihmc:ihmc-graphics-libgdx:source")
}

testDependencies {
   api(ihmc.sourceSetProject("javafx"))
   api(ihmc.sourceSetProject("libgdx"))
   api("us.ihmc:ihmc-graphics-libgdx-test:source")
   api("us.ihmc:ihmc-path-planning-test:source")
}
configurations.all {
   exclude("log4j", "log4j")
}

