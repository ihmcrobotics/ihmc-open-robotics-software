plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.martiansoftware:jsap:2.1")
   api("org.apache.poi:poi:3.15") // I/O library for xls files.
   api("com.hierynomus:sshj:0.32.0")
   api("org.jfree:org.jfree.svg:5.0.6")
   api("com.github.sh0nk:matplotlib4j:0.5.0")
   api("com.jerolba:carpet-record:0.3.0") // Used for huggingface data exports

   api("us.ihmc:mecano-graphviz:17-0.19.3")
   api("us.ihmc:scs2-bullet-simulation:17-0.32.2")
   api("us.ihmc:scs2-mujoco-simulation:17-0.32.2")
   api("us.ihmc:ihmc_hands_ros2:0.2.2")

   api("us.ihmc:ihmc-footstep-planning:source")
   api("us.ihmc:ihmc-manipulation-planning:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")

   api("com.badlogicgames.gdx-controllers:gdx-controllers-desktop:2.2.3") // for CSG plugin
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

   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:ihmc-footstep-planning-visualizers:source")
}
