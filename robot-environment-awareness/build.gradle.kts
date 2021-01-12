import org.apache.commons.lang3.SystemUtils

buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.9")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
}

ihmc {
   loadProductProperties("../product.properties")
   description = "Robot Environment Awareness is a library meant to provide interpretation to point cloud data such as identifying planar regions that can be used to planify footsteps for a bipedal robot."
   openSource = true
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.vividsolutions:jts:1.13") // TODO Update to https://github.com/locationtech/jts
   api("org.apache.commons:commons-lang3:3.8.1")
   
   api("us.ihmc:euclid-shape:0.16.1")
   api("us.ihmc:joctomap:1.12.1")
   api("us.ihmc:ihmc-yovariables:0.9.8")
   api("us.ihmc:ihmc-javafx-toolkit:0.19.3")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-robot-models-visualizers:source")
   api("us.ihmc:ihmc-robot-data-logger:0.20.5")

   apiBytedecoNatives("javacpp", "1.5.4")
   apiBytedecoNatives("openblas", "0.3.10-1.5.4")
   apiBytedecoNatives("opencv", "4.4.0-1.5.4")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoNatives(name: String, version: String)
{
   apiBytedecoSelective("org.bytedeco:$name:$version")
   apiBytedecoSelective("org.bytedeco:$name:$version:linux-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$version:windows-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$version:macosx-x86_64")
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
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.19.5")
   api("us.ihmc:simulation-construction-set:0.21.5")
   api("us.ihmc:simulation-construction-set-tools:source")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
}
