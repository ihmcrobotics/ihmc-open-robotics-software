import org.apache.commons.lang3.SystemUtils

buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
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

   api("us.ihmc:joctomap:1.12.5")
   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-messager-javafx:0.2.0")
   api("us.ihmc:ihmc-robot-data-logger:0.30.2")
   api("us.ihmc:ihmc-ros-tools:source")

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

applicationDependencies {
   api(ihmc.sourceSetProject("main"))
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
}
