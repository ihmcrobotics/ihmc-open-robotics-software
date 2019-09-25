plugins {
   id("us.ihmc.ihmc-build") version "0.19.5"
   id("us.ihmc.ihmc-ci") version "5.0"
   id("us.ihmc.ihmc-cd") version "1.7"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools") version "0.3.1"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.georegression:georegression:0.11")
   api("org.ddogleg:ddogleg:0.7")
   api("org.apache.xmlgraphics:batik-dom:1.7")
   api("org.apache.ant:ant:1.9.0")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("com.martiansoftware:jsap:2.1")
   api("org.ros.rosjava_core:rosjava:0.2.1")
   api("org.ejml:core:0.30")
   api("org.ejml:dense64:0.30")
   api("org.boofcv:io:0.24.1")
   api("org.boofcv:ip:0.24.1")
   api("org.boofcv:geo:0.24.1")
   api("org.boofcv:calibration:0.24.1")
   api("org.boofcv:visualize:0.24.1")
   api("org.ros.rosjava_bootstrap:message_generation:0.2.1")
   api("com.github.wendykierp:JTransforms:3.1")
   api("org.ros.rosjava_messages:sensor_msgs:1.11.7")
   api("org.ros.rosjava_messages:multisense_ros:3.4.2")
   api("org.bytedeco:javacv-platform:1.5")
   api("org.ros.rosjava_messages:geometry_msgs:1.11.9")
   api("org.boofcv:recognition:0.24.1")

   api("us.ihmc:euclid:0.12.1")
   api("us.ihmc:ihmc-yovariables:0.3.11")
   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.12.8")
   api("us.ihmc:simulation-construction-set:0.12.15")
   api("us.ihmc:ihmc-graphics-description:0.12.12")
   api("us.ihmc:ihmc-robot-description:0.12.7")
   api("us.ihmc:ihmc-javafx-toolkit:0.12.9")
   api("us.ihmc:ihmc-humanoid-behaviors:source")
   api("us.ihmc:ihmc-mocap:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-perception:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-robot-data-logger:source")
   api("us.ihmc:robotiq-hand-drivers:source")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:ihmc-manipulation-planning:source")
   api("us.ihmc:ihmc-parameter-tuner:source")
   api("us.ihmc:ihmc-footstep-planning-visualizers:source")
   api("us.ihmc:ihmc-avatar-interfaces-behavior-clean-room:source")
   api("us.ihmc:ihmc-avatar-interfaces-behavior-fx-ui:source")
}

testDependencies {
   api("com.thoughtworks.xstream:xstream:1.4.7")

   api("us.ihmc:ihmc-commons-testing:0.26.6")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
   api("us.ihmc:ihmc-avatar-interfaces-test:source")
   api("us.ihmc:ihmc-humanoid-robotics-test:source")
   api("us.ihmc:ihmc-sensor-processing-test:source")
   api("us.ihmc:ihmc-simulation-toolkit-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
