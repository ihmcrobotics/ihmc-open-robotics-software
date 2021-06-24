plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.20"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
}

testDependencies {

}

javafxDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-javafx-toolkit:0.19.3")
   api("us.ihmc:simulation-construction-set-tools:source")
}

javafxTestDependencies {
   api(ihmc.sourceSetProject("javafx"))
}

jmonkeyengineDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set-tools:source")
}

jmonkeyengineTestDependencies {
   api(ihmc.sourceSetProject("jmonkeyengine"))
}

libgdxDependencies {
   api(ihmc.sourceSetProject("main"))

   val gdxVersion = "1.10.0"
   api("com.badlogicgames.gdx:gdx-backend-lwjgl3:$gdxVersion")
   api("com.badlogicgames.gdx:gdx-platform:$gdxVersion:natives-desktop")

   val lwjglVersion = "3.2.3"
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-linux")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-windows")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-windows-x86")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-macos")

   val imguiVersion = "1.83.1"
   api("io.github.spair:imgui-java-binding:$imguiVersion")
   api("io.github.spair:imgui-java-lwjgl3:$imguiVersion")
   api("io.github.spair:imgui-java-natives-linux-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-macos-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-windows-ft:$imguiVersion")
}

libgdxTestDependencies {
   api(ihmc.sourceSetProject("libgdx"))
}
