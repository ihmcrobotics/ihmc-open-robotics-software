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

mainDependencies {
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-graphics-description:0.20.2")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
}

testDependencies {

}

javafxDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.2")
   api("us.ihmc:simulation-construction-set-tools:source")
}

javafxTestDependencies {
   api(ihmc.sourceSetProject("javafx"))
}

jmonkeyengineDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set-tools:source")

   var javaFXVersion = "17.0.2"
   api(ihmc.javaFXModule("graphics", javaFXVersion)) // JFX Color
}

jmonkeyengineTestDependencies {
   api(ihmc.sourceSetProject("jmonkeyengine"))
}

libgdxDependencies {
   api(ihmc.sourceSetProject("main"))

   val gdxVersion = "1.11.0"
   api("com.badlogicgames.gdx:gdx-backend-lwjgl3:$gdxVersion")
   api("com.badlogicgames.gdx:gdx-platform:$gdxVersion:natives-desktop")

   val lwjglVersion = "3.3.1"
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-linux")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-windows")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-windows-x86")
   api("org.lwjgl:lwjgl-openvr:$lwjglVersion:natives-macos")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-linux")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-windows")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-windows-x86")
   api("org.lwjgl:lwjgl-assimp:$lwjglVersion:natives-macos")

   val imguiVersion = "1.86.4"
   api("io.github.spair:imgui-java-binding:$imguiVersion")
   api("io.github.spair:imgui-java-lwjgl3:$imguiVersion")
   api("io.github.spair:imgui-java-natives-linux-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-macos-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-windows-ft:$imguiVersion")

   var javaFXVersion = "17.0.2"
   api(ihmc.javaFXModule("graphics", javaFXVersion)) // JFX Color

   api("us.ihmc:ihmc-perception-javacv:source")
}

libgdxTestDependencies {
   api(ihmc.sourceSetProject("libgdx"))
}
