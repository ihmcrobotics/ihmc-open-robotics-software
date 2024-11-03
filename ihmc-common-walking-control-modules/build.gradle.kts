plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
      
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-convex-optimization:0.17.21")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-parameter-estimation:source")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-convex-optimization-test:0.17.21")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:simulation-construction-set:0.25.1")
   api("us.ihmc:scs2-simulation-construction-set:17-0.27.3")

   var javaFXVersion = "17.0.9"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
}
