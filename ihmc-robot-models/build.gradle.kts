plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("javax.vecmath:vecmath:1.5.2")

   api("us.ihmc:ihmc-robot-description:0.25.4")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {

}
