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
   api("us.ihmc:ihmc-avatar-interfaces:source") {
      exclude(group = "org.openjfx")
      exclude(group = "org.jmonkeyengine")
      exclude(group = "org.lwjgl.lwjgl") // exclude lwjgl 2
   }
   api("us.ihmc:promp-java:1.0.0")
}

libgdxDependencies {
   api(ihmc.sourceSetProject("main"))
   api("org.abego.treelayout:org.abego.treelayout.core:1.0.3")
   api("us.ihmc:ihmc-graphics-libgdx:source")
   api("com.badlogicgames.gdx-controllers:gdx-controllers-core:2.2.3")
   api("com.badlogicgames.gdx-controllers:gdx-controllers-desktop:2.2.3")
   api("commons-io:commons-io:2.11.0") // IOUtils method was old version without this
}

testDependencies {
   api(ihmc.sourceSetProject("libgdx"))
   api("us.ihmc:ihmc-graphics-libgdx-test:source")
   api("us.ihmc:ihmc-path-planning-test:source")
   api("us.ihmc:scs2-examples:17-0.27.3")
   api("us.ihmc:scs2-bullet-simulation-test:17-0.27.3")
   api("us.ihmc:example-simulations:source")
}
