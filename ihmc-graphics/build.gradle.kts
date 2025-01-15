plugins {
   id("us.ihmc.ihmc-build")
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

   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.10")
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

libgdxDependencies {
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")

   val gdxVersion = "1.12.1"
   api("com.badlogicgames.gdx:gdx-backend-lwjgl3:$gdxVersion")
   api("com.badlogicgames.gdx:gdx-platform:$gdxVersion:natives-desktop")
   api("com.github.mgsx-dev.gdx-gltf:core:2.2.1")

   val lwjglVersion = "3.3.3"
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

   val imguiVersion = "1.86.11"
   api("io.github.spair:imgui-java-binding:$imguiVersion")
   api("io.github.spair:imgui-java-lwjgl3:$imguiVersion")
   api("io.github.spair:imgui-java-natives-linux-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-macos-ft:$imguiVersion")
   api("io.github.spair:imgui-java-natives-windows-ft:$imguiVersion")

   val javaFXVersion = "17.0.9"
   api(ihmc.javaFXModule("graphics", javaFXVersion)) // JFX Color

   val openblasVersion = "0.3.28-1.5.11"
   api("us.ihmc:openblas:$openblasVersion")
   api("us.ihmc:openblas:$openblasVersion:linux-x86_64")
   api("us.ihmc:openblas:$openblasVersion:linux-arm64")
   api("us.ihmc:openblas:$openblasVersion:windows-x86_64")
   val opencvVersion = "4.10.0-1.5.11"
   api("us.ihmc:opencv:$opencvVersion")
   api("us.ihmc:opencv:$opencvVersion:linux-arm64")
   api("us.ihmc:opencv:$opencvVersion:linux-x86_64")
   api("us.ihmc:opencv:$opencvVersion:linux-x86_64-gpu")
   api("us.ihmc:opencv:$opencvVersion:windows-x86_64")
   api("us.ihmc:opencv:$opencvVersion:windows-x86_64-gpu")
}

libgdxTestDependencies {
   api(ihmc.sourceSetProject("libgdx"))
}
