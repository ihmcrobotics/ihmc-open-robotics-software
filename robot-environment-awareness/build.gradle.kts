import org.apache.commons.lang3.SystemUtils

buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
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

mainDependencies {
   api("com.vividsolutions:jts:1.13") // TODO Update to https://github.com/locationtech/jts

   api("us.ihmc:euclid-shape:0.21.0")
   api("us.ihmc:joctomap:1.12.5")
   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.8")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-robot-data-logger:0.28.7")
   api("us.ihmc:ihmc-ros-tools:source")

   api("org.bytedeco:javacpp:1.5.9")
   val openblasVersion = "0.3.23-1.5.9"
   api("org.bytedeco:openblas:$openblasVersion")
   api("org.bytedeco:openblas:$openblasVersion:linux-x86_64")
   api("org.bytedeco:openblas:$openblasVersion:windows-x86_64")
   val opencvVersion = "4.7.0-1.5.9"
   api("org.bytedeco:opencv:$opencvVersion")
   api("org.bytedeco:opencv:$opencvVersion:linux-x86_64")
   api("org.bytedeco:opencv:$opencvVersion:windows-x86_64")
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
