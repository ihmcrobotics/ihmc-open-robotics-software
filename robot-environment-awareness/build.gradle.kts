import org.apache.commons.lang3.SystemUtils

buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   description = "Robot Environment Awareness is a library meant to provide interpretation to point cloud data such as identifying planar regions that can be used to planify footsteps for a bipedal robot."
   openSource = true
   
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   configurePublications()
}

val javaCPPVersion = "1.5.9"

mainDependencies {
   api("com.vividsolutions:jts:1.13") // TODO Update to https://github.com/locationtech/jts

   api("us.ihmc:euclid-shape:0.19.1")
   api("us.ihmc:joctomap:1.12.4")
   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.2")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-robot-data-logger:0.27.3")
   api("us.ihmc:ihmc-ros-tools:source")

   apiBytedecoNatives("javacpp", "", "-20230222.151859-137")
   apiBytedecoNatives("openblas", "0.3.21-", "-20221104.072840-16")
   apiBytedecoNatives("opencv", "4.7.0-", "-20230218.054119-148")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoNatives(name: String, versionPrefix: String = "", versionSuffix: String = "")
{
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion$versionSuffix")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion$versionSuffix:linux-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion$versionSuffix:windows-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion$versionSuffix:macosx-x86_64")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoSelective(dependencyNotation: String)
{
   api(dependencyNotation) {
      exclude(group = "org.bytedeco")
   }
}

applicationDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:simulation-construction-set-tools:source")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
}
