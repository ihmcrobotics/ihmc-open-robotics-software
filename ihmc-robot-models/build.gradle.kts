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
   api("org.apache.commons:commons-lang3:3.8.1")
   api("javax.vecmath:vecmath:1.5.2")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:ihmc-yovariables:0.9.8")
   api("us.ihmc:ihmc-robot-description:0.21.1")
   api("us.ihmc:ihmc-graphics-description:0.19.3")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:ihmc-javafx-toolkit:0.19.3")
   api("us.ihmc:simulation-construction-set:0.21.5")
}

gdxDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-graphics-description:0.19.3")
   api("com.badlogicgames.gdx:gdx-backend-lwjgl3:1.9.12")
   api("com.badlogicgames.gdx:gdx-platform:1.9.12:natives-desktop")

   api("org.lwjgl:lwjgl-openvr:3.2.3")
   api("org.lwjgl:lwjgl-openvr:3.2.3:natives-linux")
   api("org.lwjgl:lwjgl-openvr:3.2.3:natives-windows")
   api("org.lwjgl:lwjgl-openvr:3.2.3:natives-windows-x86")
   api("org.lwjgl:lwjgl-openvr:3.2.3:natives-macos")

   api("io.imgui.java:imgui-java-binding:1.79-1.4.0")
   api("io.imgui.java:imgui-java-lwjgl3:1.79-1.4.0")
   api("io.imgui.java:imgui-java-natives-linux:1.79-1.4.0")
   api("io.imgui.java:imgui-java-natives-linux-x86:1.79-1.4.0")
   api("io.imgui.java:imgui-java-natives-macos:1.79-1.4.0")
   api("io.imgui.java:imgui-java-natives-windows:1.79-1.4.0")
   api("io.imgui.java:imgui-java-natives-windows-x86:1.79-1.4.0")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
}

gdxTestDependencies {
   api(ihmc.sourceSetProject("gdx"))
}
