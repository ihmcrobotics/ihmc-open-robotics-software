plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")

}

testDependencies {
   api("us.ihmc:ihmc-common-walking-control-modules-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
}
