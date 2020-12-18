import org.apache.commons.lang3.SystemUtils

buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.9")
   }
}

plugins {
   id("us.ihmc.ihmc-build") version "0.22.0"
   id("us.ihmc.ihmc-ci") version "6.4"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   configureDependencyResolution()
   configurePublications()
}

repositories {
   maven {
      url = uri("https://oss.sonatype.org/content/repositories/snapshots")
   }
}


mainDependencies {
//   api(ihmc.sourceSetProject("javacv"))
   api(files("/usr/local/share/java/opencv4/opencv-440.jar"))

//   api("org.bytedeco:librealsense2-platform:2.38.1-1.5.5-SNAPSHOT-linux-x86_64")
//   api("org.bytedeco:javacv-platform:1.5.5-SNAPSHOT-linux-x86_64")


   api("org.apache.commons:commons-lang3:3.8.1")
   api("us.ihmc:ihmc-native-library-loader:1.2.1")
   api("org.georegression:georegression:0.22")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("net.java.dev.jna:jna:4.1.0")
   api("org.boofcv:boofcv-geo:0.36")
   api("org.boofcv:boofcv-ip:0.36")
   api("org.boofcv:boofcv-swing:0.36")
   api("org.boofcv:boofcv-io:0.36")
   api("org.boofcv:boofcv-recognition:0.36")
   api("org.boofcv:boofcv-calibration:0.36")
   api("us.ihmc.ihmcPerception:valvenet:0.0.4")
   api("us.ihmc.ihmcPerception:cuda:7.5")
   api("org.ddogleg:ddogleg:0.18")

   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:ihmc-yovariables:0.9.6")
   api("us.ihmc:simulation-construction-set:0.20.6")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.19.1")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-javafx-toolkit:0.18.0")
   api("us.ihmc:joctomap:1.11.0")
   api("us.ihmc:ihmc-path-planning:source")
   api("us.ihmc:ihmc-footstep-planning:source")
   api("us.ihmc:robot-environment-awareness:source")
}

openpnpDependencies {
   api("org.openpnp:opencv:4.3.0-2")
}

bytedecoDependencies {
   api("org.bytedeco:opencv-platform:4.4.0-1.5.4")
}

javacvDependencies {
   api("org.bytedeco:javacv-platform:1.5.4")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.3")
   api("us.ihmc:simulation-construction-set:0.20.6")
   api("us.ihmc:simulation-construction-set-test:0.20.6")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
