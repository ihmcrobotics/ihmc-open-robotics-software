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
   api("net.sf.trove4j:trove4j:3.0.3")
   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:ihmc-matrix-library:0.19.0")
}

testDependencies {
   api("org.ejml:ejml-ddense:0.39")
}

examplesDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:euclid-geometry:0.21.0")
   api("us.ihmc:ihmc-yovariables:0.12.2")
   api("us.ihmc:simulation-construction-set:0.25.1")
}

examplesTestDependencies{
   api(ihmc.sourceSetProject("examples"))
}

