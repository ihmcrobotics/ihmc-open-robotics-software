buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.9")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api(ihmc.sourceSetProject("javacv"))
   // For experimenting with local OpenCV:
   // api(files("/usr/local/share/OpenCV/java/opencv-310.jar"))

   api("org.apache.commons:commons-lang3:3.8.1")
   api("us.ihmc:ihmc-native-library-loader:1.3.1")
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

   api("us.ihmc:euclid:0.16.1")
   api("us.ihmc:ihmc-yovariables:0.9.8")
   api("us.ihmc:simulation-construction-set:0.21.5")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.19.5")
   api("us.ihmc:ihmc-graphics-description:0.19.3")
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

val javaCVVersion = "1.5.4"

javacvDependencies {
   apiExcludingBytedeco("org.bytedeco:javacv-platform:$javaCVVersion")
   apiExcludingBytedeco("org.bytedeco:javacv:$javaCVVersion")
   includeBytedecoForOSes("javacpp")
   includeBytedecoForOSes("openblas", "0.3.10-")
   includeBytedecoForOSes("opencv", "4.4.0-")

//   includeJavaCVForOS("linux-x86_64")
//   includeJavaCVForOS("windows-x86_64")
//   includeJavaCVForOS("macosx-x86_64")
}

fun us.ihmc.build.IHMCDependenciesExtension.includeBytedecoForOSes(name: String, versionPrefix: String = "")
{
   apiExcludingBytedeco("org.bytedeco:$name:$versionPrefix$javaCVVersion")
   apiExcludingBytedeco("org.bytedeco:$name:$versionPrefix$javaCVVersion:linux-x86_64")
   apiExcludingBytedeco("org.bytedeco:$name:$versionPrefix$javaCVVersion:windows-x86_64")
   apiExcludingBytedeco("org.bytedeco:$name:$versionPrefix$javaCVVersion:macosx-x86_64")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiExcludingBytedeco(dependencyNotation: String)
{
   api(dependencyNotation) {
      exclude(group = "org.bytedeco")
   }
}

fun us.ihmc.build.IHMCDependenciesExtension.includeJavaCVForOS(os: String)
{
   api("org.bytedeco:javacv-platform:$javaCVVersion:$os") {
      excludeBytedecoDependency("ffmpeg")
      excludeBytedecoDependency("flycapture")
      excludeBytedecoDependency("libdc1394")
      excludeBytedecoDependency("libfreenect")
      excludeBytedecoDependency("libfreenect2")
      excludeBytedecoDependency("librealsense")
      excludeBytedecoDependency("librealsense2")
      excludeBytedecoDependency("videoinput")
      excludeBytedecoDependency("artoolkitplus")
      excludeBytedecoDependency("flandmark")
      excludeBytedecoDependency("leptonica")
      excludeBytedecoDependency("tesseract")
   }
}

fun ExternalModuleDependency.excludeBytedecoDependency(name: String)
{
   exclude(group = "org.bytedeco", module = name)
   exclude(group = "org.bytedeco", module = "$name-platform")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.4")
   api("us.ihmc:simulation-construction-set:0.21.5")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
