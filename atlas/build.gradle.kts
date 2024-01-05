plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.georegression:georegression:0.22")
   api("org.ddogleg:ddogleg:0.18")
   api("org.apache.xmlgraphics:batik-dom:1.14")
   api("org.apache.ant:ant:1.9.0")
   api("com.martiansoftware:jsap:2.1")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.boofcv:boofcv-io:0.36")
   api("org.boofcv:boofcv-ip:0.36")
   api("org.boofcv:boofcv-geo:0.36")
   api("org.boofcv:boofcv-calibration:0.36")
   api("org.boofcv:boofcv-swing:0.36")
   api("org.boofcv:boofcv-recognition:0.36")
   api("com.github.wendykierp:JTransforms:3.1")

   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:euclid-shape:0.21.0")
   api("us.ihmc:euclid-frame-shape:0.21.0")
   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.8")
   api("us.ihmc:ihmc-humanoid-behaviors:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-simple-whole-body-walking:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
   api("us.ihmc:ihmc-avatar-interfaces-visualizers:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-perception:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
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
   api("us.ihmc:ihmc-high-level-behaviors:source")
}

testDependencies {
   api("com.thoughtworks.xstream:xstream:1.4.19")

   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
   api("us.ihmc:ihmc-avatar-interfaces-test:source")
   api("us.ihmc:ihmc-humanoid-robotics-test:source")
   api("us.ihmc:ihmc-sensor-processing-test:source")
   api("us.ihmc:ihmc-simulation-toolkit-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-messager-test:0.2.0")
}
