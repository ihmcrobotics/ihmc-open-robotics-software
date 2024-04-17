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
   configurePublications()
}

mainDependencies {
   api("com.martiansoftware:jsap:2.1")
   api("org.apache.poi:poi:3.15") // I/O library for xls files.
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.boofcv:boofcv-geo:0.36")
   api("org.jfree:jfreechart:1.0.19")
   api("com.github.quickhull3d:quickhull3d:1.0.0")
   api("com.github.wendykierp:JTransforms:3.1")
   api("org.reflections:reflections:0.9.10")
   api("com.hierynomus:sshj:0.32.0")

   api("us.ihmc:jinput:2.0.6-ihmc2")
   api("us.ihmc:euclid:0.21.0")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:mecano-graphviz:17-0.18.1")
   api("us.ihmc:robot-environment-awareness:source")
   api("us.ihmc:robot-environment-awareness-visualizers:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-humanoid-behaviors:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-perception:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-state-estimation:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:ihmc-footstep-planning:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:scs2-definition:17-0.25.0")
   api("us.ihmc:scs2-bullet-simulation:17-0.23.1")
   api("us.ihmc:scs2-session-visualizer-jfx:17-0.23.1")
   api("us.ihmc:scs2-simulation-construction-set:17-0.23.1")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))

   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-footstep-planning-test:source")
   api("us.ihmc:ihmc-humanoid-robotics-test:source")
   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.9")
   api("us.ihmc:robot-environment-awareness-application:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:ihmc-footstep-planning-visualizers:source")
}
