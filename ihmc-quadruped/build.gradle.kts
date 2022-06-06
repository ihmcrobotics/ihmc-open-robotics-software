plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

basicsDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:jinput:2.0.6-ihmc2")
   api("us.ihmc:euclid-frame:0.17.2")
   api("us.ihmc:euclid-frame-shape:0.17.2")
   api("us.ihmc:euclid-shape:0.17.2")
   api("us.ihmc:ihmc-robot-description:0.21.4")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
}

basicsTestDependencies {
   api(ihmc.sourceSetProject("basics"))

   api("com.google.caliper:caliper:1.0-beta-2")

   api("us.ihmc:simulation-construction-set-tools-test:source")
}

planningDependencies {
   api(ihmc.sourceSetProject("basics"))

   api("us.ihmc:robot-environment-awareness:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robot-data-logger:0.23.0")
   api("us.ihmc:ihmc-path-planning:source")
}

planningTestDependencies {
   api(ihmc.sourceSetProject("planning"))
}

footstepPlanningDependencies {
   api(ihmc.sourceSetProject("planning"))

   api("us.ihmc:ihmc-path-planning-data-sets:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
}

footstepPlanningVisualizersDependencies {
   api(ihmc.sourceSetProject("footstep-planning"))

   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
}

footstepPlanningTestDependencies {
   api(ihmc.sourceSetProject("footstep-planning-visualizers"))
}

communicationDependencies {
   api(ihmc.sourceSetProject("footstep-planning"))

   api("us.ihmc:euclid-geometry:0.17.2")
}

communicationTestDependencies {
   api(ihmc.sourceSetProject("communication"))
   api(ihmc.sourceSetProject("footstep-planning-visualizers"))
}

roboticsDependencies {
   api(ihmc.sourceSetProject("communication"))

   api("us.ihmc:ihmc-convex-optimization:0.17.11")
   api("us.ihmc:ihmc-state-estimation:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
   api("us.ihmc:ihmc-system-identification:source")
}

roboticsTestDependencies {
   api(ihmc.sourceSetProject("robotics"))
   api(ihmc.sourceSetProject("basics-test"))

   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
   api("us.ihmc:ihmc-communication-test:source")
}

uiDependencies {
   api(ihmc.sourceSetProject("robotics"))
   api(ihmc.sourceSetProject("footstep-planning-visualizers"))
}

uiTestDependencies {
   api(ihmc.sourceSetProject("ui"))
}
