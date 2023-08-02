plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   configurePublications();
}

mainDependencies {
   api("org.bytedeco:javacpp:1.5.9")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-native-library-loader:2.0.2")
}
