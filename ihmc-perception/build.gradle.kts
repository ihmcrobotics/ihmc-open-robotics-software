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
   javaDirectory("slam-wrapper", "generated-java")
   configurePublications()
}

mainDependencies {
   api(ihmc.sourceSetProject("javacv"))
   // For experimenting with local OpenCV:
   // api(files("/usr/local/share/OpenCV/java/opencv-310.jar"))

   api("us.ihmc:ihmc-native-library-loader:2.0.1")
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

   api("us.ihmc:euclid:0.19.0")
   api("us.ihmc:simulation-construction-set:0.22.10")
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

val javaCPPVersion = "1.5.7"

bytedecoDependencies {
   api("us.ihmc:euclid:0.18.1")
   api("us.ihmc:ihmc-commons:0.31.0")
   apiBytedecoNatives("javacpp")
   apiBytedecoNatives("openblas", "0.3.19-")
   apiBytedecoNatives("opencv", "4.5.5-")
   apiBytedecoNatives("opencl", "3.0-")
   apiBytedecoNatives("librealsense2", "2.50.0-")
   apiBytedecoNatives("spinnaker", "2.4.0.143-")
   apiBytedecoNatives("ffmpeg", "5.0-")
   apiBytedecoNatives("hdf5", "1.12.1-")
}

javacvDependencies {
   apiBytedecoSelective("org.bytedeco:javacv:$javaCPPVersion")
   apiBytedecoNatives("javacpp")
   apiBytedecoNatives("openblas", "0.3.19-")
   apiBytedecoNatives("opencv", "4.5.5-")
   apiBytedecoNatives("opencl", "3.0-")
   apiBytedecoNatives("librealsense2", "2.50.0-")
   apiBytedecoNatives("spinnaker", "2.4.0.143-")
   apiBytedecoNatives("ffmpeg", "5.0-")
   apiBytedecoNatives("hdf5", "1.12.1-")
}

slamWrapperDependencies {
   apiBytedecoNatives("javacpp")
   api("us.ihmc:ihmc-java-toolkit:source")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoNatives(name: String, versionPrefix: String = "")
{
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:linux-x86_64")
   if (name != "spinnaker")
      apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:linux-arm64")
   apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:windows-x86_64")
   if (name != "spinnaker")
      apiBytedecoSelective("org.bytedeco:$name:$versionPrefix$javaCPPVersion:macosx-x86_64")
}

fun us.ihmc.build.IHMCDependenciesExtension.apiBytedecoSelective(dependencyNotation: String)
{
   api(dependencyNotation) {
      exclude(group = "org.bytedeco")
   }
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set:0.22.10")

   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

tasks.create("generateMappings", Exec::class)
{
   workingDir = file("src/slam-wrapper/cpp")
   commandLine = listOf("./generate-java-mappings-docker.sh")
}


