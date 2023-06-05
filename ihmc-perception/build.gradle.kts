buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.24"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   javaDirectory("slam-wrapper", "generated-java")
   javaDirectory("mapsense-wrapper", "generated-java")
   configurePublications()
}

mainDependencies {
   api(ihmc.sourceSetProject("slam-wrapper"))
   api(ihmc.sourceSetProject("mapsense-wrapper"))
   // For experimenting with local OpenCV:
   // api(files("/usr/local/share/OpenCV/java/opencv-310.jar"))

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

   api("us.ihmc:javacpp:1.5.9")
   val openblasVersion = "0.3.23-1.5.9"
   api("us.ihmc:openblas:$openblasVersion")
   api("us.ihmc:openblas:$openblasVersion:linux-x86_64")
   api("us.ihmc:openblas:$openblasVersion:windows-x86_64")
   val opencvVersion = "4.7.0-1.5.9"
   api("us.ihmc:opencv:$opencvVersion")
   api("us.ihmc:opencv:$opencvVersion:linux-x86_64")
   api("us.ihmc:opencv:$opencvVersion:windows-x86_64")
   val ffmpegVersion = "6.1-1.5.9"
   api("us.ihmc:ffmpeg:$ffmpegVersion")
   api("us.ihmc:ffmpeg:$ffmpegVersion:linux-x86_64")
   api("us.ihmc:ffmpeg:$ffmpegVersion:windows-x86_64")
   val openclVersion = "3.1-1.5.9"
   api("us.ihmc:opencl:$openclVersion")
   api("us.ihmc:opencl:$openclVersion:linux-x86_64")
   api("us.ihmc:opencl:$openclVersion:windows-x86_64")
   val librealsense2Version = "2.53.1-1.5.9"
   api("us.ihmc:librealsense2:$librealsense2Version")
   api("us.ihmc:librealsense2:$librealsense2Version:linux-x86_64")
   api("us.ihmc:librealsense2:$librealsense2Version:windows-x86_64")
   val spinnakerVersion = "3.0.0.118-1.5.9"
   api("us.ihmc:spinnaker:$spinnakerVersion")
   api("us.ihmc:spinnaker:$spinnakerVersion:linux-x86_64")
   api("us.ihmc:spinnaker:$spinnakerVersion:windows-x86_64")
   val hdf5Version = "1.14.1-1.5.9"
   api("us.ihmc:hdf5:$hdf5Version")
   api("us.ihmc:hdf5:$hdf5Version:linux-x86_64")
   api("us.ihmc:hdf5:$hdf5Version:windows-x86_64")

   api("us.ihmc:euclid:0.20.0")
   api("us.ihmc:simulation-construction-set:0.23.4")
   api("us.ihmc:ihmc-native-library-loader:2.0.2")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:robot-environment-awareness:source")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

slamWrapperDependencies {
   api("us.ihmc:javacpp:1.5.9")
   api("us.ihmc:ihmc-java-toolkit:source")
}

mapsenseWrapperDependencies {
   api("us.ihmc:javacpp:1.5.9")
   api("us.ihmc:ihmc-java-toolkit:source")
}
