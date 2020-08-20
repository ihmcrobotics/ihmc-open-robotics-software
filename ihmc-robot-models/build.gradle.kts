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

mainDependencies {
   api("org.apache.commons:commons-lang3:3.8.1")
   api("javax.vecmath:vecmath:1.5.2")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:ihmc-yovariables:0.9.3")
   api("us.ihmc:ihmc-robot-description:0.20.1")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:ihmc-javafx-toolkit:0.19.1")
   api("us.ihmc:simulation-construction-set:0.20.4")
}

gdxDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
   api("com.badlogicgames.gdx:gdx-backend-lwjgl3:1.9.10")
   api("com.badlogicgames.gdx:gdx-platform:1.9.10:natives-desktop")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
}

gdxTestDependencies {
   api(ihmc.sourceSetProject("gdx"))
}
