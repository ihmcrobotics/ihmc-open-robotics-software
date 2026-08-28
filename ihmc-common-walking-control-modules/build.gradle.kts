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
   api("us.ihmc:ihmc-convex-optimization:0.18.0")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-parameter-estimation:source")
   api("us.ihmc:ihmc-robotics-tools:0.15.7")
   api("us.ihmc:mecano-yovariables-filters:17-0.19.3")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-convex-optimization-test:0.18.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:simulation-construction-set:0.25.4")
   api("us.ihmc:scs2-simulation-construction-set:17-0.33.4")

   var javaFXVersion = "17.0.8"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
}
