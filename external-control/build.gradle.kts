plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-avatar-interfaces:source")
}
