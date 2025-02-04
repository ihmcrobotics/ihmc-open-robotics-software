buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
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

   api("org.georegression:georegression:0.22")
   api("net.java.dev.jna:jna:4.1.0")

   // ffmpeg, openblas, opencv come from the logger

   val cudaVersion = "12.6-9.5-1.5.11-ihmc-2"
   api("us.ihmc:cuda:$cudaVersion")
   api("us.ihmc:cuda:$cudaVersion:linux-arm64")
   api("us.ihmc:cuda:$cudaVersion:linux-x86_64")
   api("us.ihmc:cuda:$cudaVersion:windows-x86_64")
   val openclVersion = "3.0-1.5.11-ihmc-2"
   api("us.ihmc:opencl:$openclVersion")
   api("us.ihmc:opencl:$openclVersion:linux-arm64")
   api("us.ihmc:opencl:$openclVersion:linux-x86_64")
   api("us.ihmc:opencl:$openclVersion:windows-x86_64")
   val spinnakerVersion = "4.0.0.116-1.5.11-ihmc-2"
   api("us.ihmc:spinnaker:$spinnakerVersion")
   api("us.ihmc:spinnaker:$spinnakerVersion:linux-x86_64")
   api("us.ihmc:spinnaker:$spinnakerVersion:windows-x86_64")
   val librealsense2Version = "2.53.1-1.5.11-ihmc-2"
   api("us.ihmc:librealsense2:$librealsense2Version")
   api("us.ihmc:librealsense2:$librealsense2Version:linux-arm64")
   api("us.ihmc:librealsense2:$librealsense2Version:linux-x86_64")
   api("us.ihmc:librealsense2:$librealsense2Version:windows-x86_64")
   val hdf5Version = "1.14.3-1.5.11-ihmc-2"
   api("us.ihmc:hdf5:$hdf5Version")
   api("us.ihmc:hdf5:$hdf5Version:linux-x86_64")
   api("us.ihmc:hdf5:$hdf5Version:windows-x86_64")

   api("us.ihmc:zed-java-api:4.2.0_2")

   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:robot-environment-awareness:source")

   // Previously used for HeightMapAutoencoder and FootstepPredictor
   // This is a very large dependency, only uncomment for testing purposes
   // api("com.microsoft.onnxruntime:onnxruntime:1.11.0")
   // api("com.microsoft.onnxruntime:onnxruntime_gpu:1.11.0")
}

testDependencies {
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

slamWrapperDependencies {
   api("us.ihmc:javacpp:1.5.11-ihmc-2")
   api("us.ihmc:ihmc-java-toolkit:source")
}
