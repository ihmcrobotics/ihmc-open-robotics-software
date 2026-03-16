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
   api("org.jgrapht:jgrapht-core:0.9.0")

   api("us.ihmc:ihmc-convex-optimization:0.18.0")
   api("us.ihmc:ihmc-perception:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("data-sets"))
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api(ihmc.sourceSetProject("data-sets"))

   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

dataSetsDependencies {
   api(ihmc.sourceSetProject("main"))
}
