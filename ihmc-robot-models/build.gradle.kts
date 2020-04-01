plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools") version "0.3.1"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.apache.commons:commons-lang3:3.8.1")
   api("javax.vecmath:vecmath:1.5.2")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:ihmc-yovariables:0.4.0")
   api("us.ihmc:ihmc-robot-description:0.14.1")
   api("us.ihmc:ihmc-graphics-description:0.14.1")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {

}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("test"))
   api("us.ihmc:ihmc-interfaces:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set:0.14.0")
}
