plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
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
   api("us.ihmc:ihmc-javafx-toolkit:0.20.0")
   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:robot-environment-awareness-visualizers:source")
}

libgdxDependencies {
   api(ihmc.sourceSetProject("main"))
   api("org.abego.treelayout:org.abego.treelayout.core:1.0.3")
   api("us.ihmc:ihmc-graphics-libgdx:source")
   api("us.ihmc:ihmc-perception-javacv:source")
   api("us.ihmc:ihmc-graphics-libgdx:source")
   api("us.ihmc:scs2-simulation:0.4.0")
   api("com.badlogicgames.gdx-controllers:gdx-controllers-core:2.2.1")
   api("com.badlogicgames.gdx-controllers:gdx-controllers-desktop:2.2.1")
   api("com.badlogicgames.gdx:gdx-bullet:1.10.0")
   api("com.badlogicgames.gdx:gdx-bullet-platform:1.10.0:natives-desktop")
}

testDependencies {
   api(ihmc.sourceSetProject("javafx"))
   api(ihmc.sourceSetProject("libgdx"))
   api("us.ihmc:ihmc-graphics-libgdx-test:source")
   api("us.ihmc:ihmc-path-planning-test:source")
   api("org.cartesiantheatrics:bag-reader-java:0.0.1")
   api("com.github.stephengold:Libbulletjme:12.6.0")
}
