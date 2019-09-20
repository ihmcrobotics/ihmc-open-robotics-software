plugins {
   id("us.ihmc.ihmc-build") version "0.18.5"
   id("us.ihmc.ihmc-ci") version "4.26"
   id("us.ihmc.ihmc-cd") version "1.4"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools") version "0.3.1"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   compile("org.georegression:georegression:0.11")
   compile("org.ddogleg:ddogleg:0.7")
   compile("org.apache.xmlgraphics:batik-dom:1.7")
   compile("org.apache.ant:ant:1.9.0")
   compile("org.apache.commons:commons-lang3:3.8.1")
   compile("com.martiansoftware:jsap:2.1")
   compile("org.ros.rosjava_core:rosjava:0.2.1")
   compile("org.ejml:core:0.30")
   compile("org.ejml:dense64:0.30")
   compile("org.boofcv:io:0.24.1")
   compile("org.boofcv:ip:0.24.1")
   compile("org.boofcv:geo:0.24.1")
   compile("org.boofcv:calibration:0.24.1")
   compile("org.boofcv:visualize:0.24.1")
   compile("org.ros.rosjava_bootstrap:message_generation:0.2.1")
   compile("com.github.wendykierp:JTransforms:3.1")
   compile("org.ros.rosjava_messages:sensor_msgs:1.11.7")
   compile("org.ros.rosjava_messages:multisense_ros:3.4.2")
   compile("org.bytedeco:javacv-platform:1.5")
   compile("org.ros.rosjava_messages:geometry_msgs:1.11.9")
   compile("org.boofcv:recognition:0.24.1")

   compile("us.ihmc:euclid:0.12.1")
   compile("us.ihmc:ihmc-yovariables:0.3.11")
   compile("us.ihmc:ihmc-commons:0.26.6")
   compile("us.ihmc:ihmc-jmonkey-engine-toolkit:0.12.8")
   compile("us.ihmc:simulation-construction-set:0.12.15")
   compile("us.ihmc:ihmc-graphics-description:0.12.12")
   compile("us.ihmc:ihmc-robot-description:0.12.7")
   compile("us.ihmc:ihmc-javafx-toolkit:0.12.9")
   compile("us.ihmc:ihmc-humanoid-behaviors:source")
   compile("us.ihmc:ihmc-mocap:source")
   compile("us.ihmc:ihmc-common-walking-control-modules:source")
   compile("us.ihmc:ihmc-avatar-interfaces:source")
   compile("us.ihmc:ihmc-humanoid-robotics:source")
   compile("us.ihmc:ihmc-communication:source")
   compile("us.ihmc:ihmc-java-toolkit:source")
   compile("us.ihmc:ihmc-perception:source")
   compile("us.ihmc:ihmc-robotics-toolkit:source")
   compile("us.ihmc:ihmc-ros-tools:source")
   compile("us.ihmc:ihmc-whole-body-controller:source")
   compile("us.ihmc:ihmc-robot-data-logger:source")
   compile("us.ihmc:robotiq-hand-drivers:source")
   compile("us.ihmc:ihmc-model-file-loader:source")
   compile("us.ihmc:ihmc-sensor-processing:source")
   compile("us.ihmc:simulation-construction-set-tools:source")
   compile("us.ihmc:ihmc-robot-models:source")
   compile("us.ihmc:ihmc-simulation-toolkit:source")
   compile("us.ihmc:ihmc-robot-data-visualizer:source")
   compile("us.ihmc:ihmc-manipulation-planning:source")
   compile("us.ihmc:ihmc-parameter-tuner:source")
   compile("us.ihmc:ihmc-footstep-planning-visualizers:source")
   compile("us.ihmc:ihmc-avatar-interfaces-behavior-clean-room:source")
   compile("us.ihmc:ihmc-avatar-interfaces-behavior-fx-ui:source")
}

testDependencies {
   compile("com.thoughtworks.xstream:xstream:1.4.7")

   compile("us.ihmc:ihmc-commons-testing:0.26.6")
   compile("us.ihmc:ihmc-robotics-toolkit-test:source")
   compile("us.ihmc:ihmc-common-walking-control-modules-test:source")
   compile("us.ihmc:ihmc-avatar-interfaces-test:source")
   compile("us.ihmc:ihmc-humanoid-robotics-test:source")
   compile("us.ihmc:ihmc-sensor-processing-test:source")
   compile("us.ihmc:ihmc-simulation-toolkit-test:source")
   compile("us.ihmc:simulation-construction-set-tools-test:source")
}
