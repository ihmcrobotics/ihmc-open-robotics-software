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
   api("us.ihmc:mecano:17-0.18.1")
}

graphvizDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:mecano-graphviz:17-0.18.1")
}

yovariablesDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:mecano-yovariables:17-0.18.1")
}

yovariablesFiltersDependencies {
   api(ihmc.sourceSetProject("yovariables"))

   api("us.ihmc:ihmc-yovariables-filters:source")
}

testDependencies {
   api("us.ihmc:mecano-test:17-0.18.1")
}
