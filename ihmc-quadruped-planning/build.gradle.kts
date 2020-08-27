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
    api("us.ihmc.thirdparty.jinput:jinput:200128")
    api("org.ejml:ejml-core:0.39")
    api("org.ejml:ejml-ddense:0.39")

    api("us.ihmc:euclid-frame:0.15.1")
    api("us.ihmc:euclid-shape:0.15.1")
    api("us.ihmc:ihmc-yovariables:0.9.3")
    api("us.ihmc:ihmc-robot-description:0.20.1")
    api("us.ihmc:ihmc-robotics-toolkit:source")
    api("us.ihmc:ihmc-humanoid-robotics:source")
    api("us.ihmc:ihmc-quadruped-basics:source")
    api("us.ihmc:robot-environment-awareness:source")
    api("us.ihmc:simulation-construction-set-tools-test:source")
    api("us.ihmc:ihmc-robot-data-logger:0.20.1")
    api("us.ihmc:ihmc-path-planning:source")
}

testDependencies {

}
