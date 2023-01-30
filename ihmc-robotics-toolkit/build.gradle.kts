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
   api("com.github.quickhull3d:quickhull3d:1.0.0")
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.georegression:georegression:0.22")
   api("org.ddogleg:ddogleg:0.18")
   api("gov.nist.math:jama:1.0.3")
   api("org.apache.commons:commons-math3:3.6.1")
   api("jgraph:jgraph:5.13.0.0")
   api("org.ejml:ejml-zdense:0.39")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.javassist:javassist:3.18.1-GA")
   api("com.google.guava:guava:18.0")
   api("org.yaml:snakeyaml:1.17") //1.11

   api("us.ihmc:log-tools:0.6.3")
   api("us.ihmc:euclid:0.19.1")
   api("us.ihmc:euclid-frame:0.19.1")
   api("us.ihmc:euclid-frame-shape:0.19.1")
   api("us.ihmc:euclid-shape:0.19.1")
   api("us.ihmc:mecano:17-0.11.5")
   api("us.ihmc:mecano-yovariables:17-0.11.5")
   api("us.ihmc:ihmc-commons:0.32.0")
   api("us.ihmc:ihmc-messager-kryo:0.1.7")
   api("us.ihmc:ihmc-matrix-library:0.18.8")
   api("us.ihmc:ihmc-graphics-description:0.19.8")
   api("us.ihmc:simulation-construction-set-utilities:0.22.10")
   api("us.ihmc:ihmc-native-library-loader:2.0.2")
   api("us.ihmc:scs2-definition:17-0.11.1")
   // NOTE: IHMCRoboticsToolkit should not depend on any other IHMC project(!), especially it should not depend on IHMCJavaToolkit.
}

testDependencies {
   api("org.jfree:jfreechart:1.0.19")
   api("org.jfree:jcommon:1.0.24")

   api("us.ihmc:euclid-test:0.19.1")
   api("us.ihmc:simulation-construction-set:0.22.10")
   api("us.ihmc:ihmc-matrix-library-test:0.18.8")
   api("us.ihmc:ihmc-commons-testing:0.32.0")
}
