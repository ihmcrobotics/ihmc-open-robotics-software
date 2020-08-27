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
   api("org.apache.commons:commons-lang3:3.8.1")
   api("commons-io:commons-io:2.6")
   api("org.apache.lucene:lucene-analyzers-common:4.3.0")
   api("org.apache.lucene:lucene-core:4.3.0")
   api("org.apache.lucene:lucene-queryparser:4.3.0")
   api("us.ihmc.thirdparty.jinput:jinput:200128")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:ihmc-commons:0.30.3")
   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:euclid-frame:0.15.1")
   api("us.ihmc:euclid-shape:0.15.1")
   api("us.ihmc:euclid-geometry:0.15.1")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
