import org.apache.commons.lang3.SystemUtils

buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.9")
   }
}

plugins {
   id("us.ihmc.ihmc-build") version "0.21.0"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
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
   api("us.ihmc:ihmc-yovariables:0.9.3")
   api("us.ihmc:simulation-construction-set:0.20.4")
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
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.3")
   api("us.ihmc:simulation-construction-set:0.20.4")
   api("us.ihmc:simulation-construction-set-test:0.20.4")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
