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
   api("com.hierynomus:sshj:0.32.0")

   api("us.ihmc:mecano-graphviz:17-0.18.1")
   api("us.ihmc:scs2-bullet-simulation:17-0.27.3")

   api("us.ihmc:ihmc-humanoid-behaviors:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:robot-environment-awareness-visualizers:source")
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

   api("us.ihmc:robot-environment-awareness-application:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:ihmc-footstep-planning-visualizers:source")
}
