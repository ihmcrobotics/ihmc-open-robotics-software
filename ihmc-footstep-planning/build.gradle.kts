plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid:0.16.1")
   api("us.ihmc:ihmc-yovariables:0.9.8")
   api("us.ihmc:ihmc-graphics-description:0.19.3")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-convex-optimization:0.17.3")
   api("us.ihmc:ihmc-path-planning:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-path-planning-data-sets:source")
   api("us.ihmc:ihmc-pub-sub-serializers-extra:0.15.0")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))

   api("us.ihmc:ihmc-path-planning-data-sets:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-path-planning-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-javafx-toolkit:0.19.3")
   api("us.ihmc:robot-environment-awareness-application:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-robot-models-visualizers:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-common-walking-control-modules-test:source")

}
