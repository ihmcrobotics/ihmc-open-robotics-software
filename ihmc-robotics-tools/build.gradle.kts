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
   api("us.ihmc:ihmc-commons-utils:source")
   api("us.ihmc:euclid-update-frame:source")
   api("us.ihmc:ihmc-yovariables:0.12.2")
   api("us.ihmc:ihmc-yovariables-filters:source")
   api("us.ihmc:ihmc-graphics-description-update:source")
}

visualizersDependencies{
   api(ihmc.sourceSetProject("main"))
   api("jgraph:jgraph:5.13.0.0")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
}