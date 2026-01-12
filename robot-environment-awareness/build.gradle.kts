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
   api("us.ihmc:ihmc-messager-javafx:0.2.1")
   api("us.ihmc:ihmc-robot-data-logger:0.36.5")
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
