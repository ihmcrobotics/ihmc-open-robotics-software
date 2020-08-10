plugins {
    id("us.ihmc.ihmc-build") version "0.21.0"
    id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
    id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
    loadProductProperties("../product.properties")

    configureDependencyResolution()
    configurePublications()
}

mainDependencies {
    api("us.ihmc:ihmc-quadruped-planning:source")
    api("us.ihmc:ihmc-path-planning-data-sets:source")
    api("us.ihmc:ihmc-common-walking-control-modules:source")

}

testDependencies {
    api(ihmc.sourceSetProject("visualizers"))
    api("us.ihmc:ihmc-quadruped-planning-test:source")
}

visualizersDependencies {
    api(ihmc.sourceSetProject("main"))

    api("us.ihmc:ihmc-robot-models-visualizers:source")
    api("us.ihmc:ihmc-path-planning-visualizers:source")
    api("us.ihmc:ihmc-common-walking-control-modules:source")
}