buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   javaDirectory("slam-wrapper", "generated-java")
   configurePublications()
}

mainDependencies {
   api(ihmc.sourceSetProject("slam-wrapper"))
   // For experimenting with local OpenCV:
   // api(files("/usr/local/share/OpenCV/java/opencv-310.jar"))

   api("org.georegression:georegression:0.22")
   api("net.java.dev.jna:jna:4.1.0")
   api("org.boofcv:boofcv-ip:0.36")
   api("org.boofcv:boofcv-swing:0.36")
   api("org.boofcv:boofcv-io:0.36")
   api("org.boofcv:boofcv-recognition:0.36")
   api("org.boofcv:boofcv-calibration:0.36")

   api("org.bytedeco:javacpp:1.5.9")
   val openblasVersion = "0.3.23-1.5.9"
   api("org.bytedeco:openblas:$openblasVersion")
   api("org.bytedeco:openblas:$openblasVersion:linux-x86_64")
   api("org.bytedeco:openblas:$openblasVersion:linux-arm64")
   api("org.bytedeco:openblas:$openblasVersion:windows-x86_64")
   val opencvVersion = "4.7.0-1.5.9"
   api("org.bytedeco:opencv:$opencvVersion")
   api("org.bytedeco:opencv:$opencvVersion:linux-x86_64")
   api("org.bytedeco:opencv:$opencvVersion:linux-arm64")
   api("org.bytedeco:opencv:$opencvVersion:windows-x86_64")
   api("org.bytedeco:opencv:$opencvVersion:linux-x86_64-gpu")
   api("org.bytedeco:opencv:$opencvVersion:windows-x86_64-gpu")
   val ffmpegVersion = "6.0-1.5.9"
   api("org.bytedeco:ffmpeg:$ffmpegVersion")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:linux-x86_64")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:linux-arm64")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:windows-x86_64")
   val openclVersion = "3.0-1.5.9"
   api("org.bytedeco:opencl:$openclVersion")
   api("org.bytedeco:opencl:$openclVersion:linux-x86_64")
   api("org.bytedeco:opencl:$openclVersion:linux-arm64")
   api("org.bytedeco:opencl:$openclVersion:windows-x86_64")
   val librealsense2Version = "2.53.1-1.5.9"
   api("org.bytedeco:librealsense2:$librealsense2Version")
   api("org.bytedeco:librealsense2:$librealsense2Version:linux-x86_64")
   api("org.bytedeco:librealsense2:$librealsense2Version:linux-arm64")
   api("org.bytedeco:librealsense2:$librealsense2Version:windows-x86_64")
   val spinnakerVersion = "3.0.0.118-1.5.9"
   api("org.bytedeco:spinnaker:$spinnakerVersion") {
      exclude(group = "org.bytedeco", module = "javacpp")
   }
   api("org.bytedeco:spinnaker:$spinnakerVersion:linux-x86_64") {
      exclude(group = "org.bytedeco", module = "javacpp")
   }
   api("org.bytedeco:spinnaker:$spinnakerVersion:windows-x86_64") {
      exclude(group = "org.bytedeco", module = "javacpp")
   }
   api("us.ihmc:zed-java-api:4.1.4") {
      exclude(group = "org.bytedeco")
   }
   val hdf5Version = "1.14.1-1.5.9"
   api("org.bytedeco:hdf5:$hdf5Version")
   api("org.bytedeco:hdf5:$hdf5Version:linux-x86_64")
   // No arm64 version
   api("org.bytedeco:hdf5:$hdf5Version:windows-x86_64")
   val cudaVersion = "12.3-8.9-1.5.10"
   api("org.bytedeco:cuda:$cudaVersion") {
      exclude(group = "org.bytedeco", module = "javacpp")
   }
   api("org.bytedeco:cuda:$cudaVersion:linux-x86_64") {
      exclude(group = "org.bytedeco", module = "javacpp")
   }
   api("org.bytedeco:cuda:$cudaVersion:linux-arm64") {
      exclude(group = "org.bytedeco", module = "javacpp")
   }
   api("org.bytedeco:cuda:$cudaVersion:windows-x86_64") {
      exclude(group = "org.bytedeco", module = "javacpp")
   }

   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:robot-environment-awareness:source")

   api("com.microsoft.onnxruntime:onnxruntime:1.11.0")
   api("com.microsoft.onnxruntime:onnxruntime_gpu:1.11.0")
}

testDependencies {
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

slamWrapperDependencies {
   api("org.bytedeco:javacpp:1.5.9")
   api("us.ihmc:ihmc-java-toolkit:source")
}
