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

   // ffmpeg, openblas, opencv, zed-java-api come from the logger

   val cudaVersion = "12.6-9.5-1.5.11"
   api("org.bytedeco:cuda:$cudaVersion")
   api("org.bytedeco:cuda:$cudaVersion:linux-arm64")
   api("org.bytedeco:cuda:$cudaVersion:linux-x86_64")
   api("org.bytedeco:cuda:$cudaVersion:windows-x86_64")
   val librealsense2Version = "2.53.1-1.5.11-ihmc-3" // https://robotlabfiles.ihmc.us/repository/us/ihmc/librealsense2/2.53.1-1.5.11-ihmc-3/
   api("us.ihmc:librealsense2:$librealsense2Version")
   api("us.ihmc:librealsense2:$librealsense2Version:linux-arm64")
   api("us.ihmc:librealsense2:$librealsense2Version:linux-x86_64")
   api("us.ihmc:librealsense2:$librealsense2Version:windows-x86_64")

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
   api("org.bytedeco:javacpp:1.5.11")
   api("us.ihmc:ihmc-java-toolkit:source")
}

app.entrypoint("IsaacROSFoundationPoseDemo", "us.ihmc.perception.demo.IsaacROSFoundationPoseDemo")
