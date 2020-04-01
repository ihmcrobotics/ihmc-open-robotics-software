import org.apache.commons.lang3.SystemUtils

buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.9")
   }
}

plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools") version "0.3.1"
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
   
   api("us.ihmc:euclid-shape:0.12.2")
   api("us.ihmc:joctomap:1.7.0")
   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:ihmc-yovariables:0.4.0")
   api("us.ihmc:ihmc-javafx-toolkit:0.14.1")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-robot-data-logger:0.15.0")

   api("org.bytedeco:javacv-platform:1.5") {
      exclude(group = "org.bytedeco", module = "opencv")
   }
   api("org.bytedeco:opencv:4.1.2-1.5.2:")
   if (SystemUtils.IS_OS_UNIX)
   {
      api("org.bytedeco:opencv:4.1.2-1.5.2:linux-x86_64")
   }
   else if (SystemUtils.IS_OS_WINDOWS)
   {
      api("org.bytedeco:opencv:4.1.2-1.5.2:windows-x86_64")
   }
   else if (SystemUtils.IS_OS_MAC_OSX)
   {
      api("org.bytedeco:opencv:4.1.2-1.5.2:macosx-x86_64")
   }
}

applicationDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.14.0")
   api("us.ihmc:simulation-construction-set:0.14.0")
   api("us.ihmc:simulation-construction-set-tools:source")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
}
