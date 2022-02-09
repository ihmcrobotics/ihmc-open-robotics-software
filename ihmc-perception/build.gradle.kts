buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
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
   api("org.ddogleg:ddogleg:0.18")

   api("us.ihmc:euclid:0.17.0")
   api("us.ihmc:ihmc-yovariables:0.9.11")
   api("us.ihmc:simulation-construction-set:0.21.15")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.19.7")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

openpnpDependencies {
   api("org.openpnp:opencv:4.3.0-2")
}

val javaCPPVersion = "1.5.6"

bytedecoDependencies {
   apiBytedecoNatives("javacpp")
   apiBytedecoNatives("openblas", "0.3.17-")
   apiBytedecoNatives("opencv", "4.5.3-")
   apiBytedecoNatives("opencl", "3.0-")
}

javacvDependencies {
   apiBytedecoSelective("org.bytedeco:javacv:$javaCPPVersion")
   apiBytedecoNatives("javacpp")
   apiBytedecoNatives("openblas", "0.3.17-")
   apiBytedecoNatives("opencv", "4.5.3-")
   apiBytedecoNatives("opencl", "3.0-")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoNatives(name: String, versionPrefix: String = "")
{
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:linux-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:windows-x86_64")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:macosx-x86_64")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoSelective(dependencyNotation: String)
{
   api(dependencyNotation) {
      exclude(group = "org.bytedeco")
   }
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.5")
   api("us.ihmc:simulation-construction-set:0.21.15")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
