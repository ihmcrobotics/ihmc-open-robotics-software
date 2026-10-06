plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
   id("us.ihmc.scs") version "0.4"
}

apply(from = "../gradle/java-compile-encoding.gradle.kts")

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:simulation-construction-set-tools:source")
}

benchmarksDependencies {
   api(ihmc.sourceSetProject("main"))
}

testDependencies {
   api("us.ihmc:ihmc-convex-optimization:0.18.0")
}
