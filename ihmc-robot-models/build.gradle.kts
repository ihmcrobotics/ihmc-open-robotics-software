plugins {
   id("us.ihmc.ihmc-build") version "0.18.5"
   id("us.ihmc.ihmc-ci") version "4.25"
   id("us.ihmc.ihmc-cd") version "0.1"
   id("us.ihmc.log-tools") version "0.3.1"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   compile("org.apache.commons:commons-lang3:3.8.1")
   compile("javax.vecmath:vecmath:1.5.2")
   compile("com.google.guava:guava:18.0")

   compile("us.ihmc:ihmc-commons:0.26.6")
   compile("us.ihmc:ihmc-yovariables:0.3.11")
   compile("us.ihmc:ihmc-robot-description:0.12.7")
   compile("us.ihmc:ihmc-graphics-description:0.12.12")
   compile("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {

}

visualizersDependencies {
   compile(ihmc.sourceSetProject("main"))
   compile(ihmc.sourceSetProject("test"))
   compile("us.ihmc:ihmc-interfaces:source")
   compile("us.ihmc:ihmc-java-toolkit:source")
   compile("us.ihmc:simulation-construction-set-tools:source")
   compile("us.ihmc:simulation-construction-set:0.12.15")
}
