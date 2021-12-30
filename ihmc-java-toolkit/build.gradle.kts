plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.5"
   id("us.ihmc.ihmc-cd") version "1.20"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
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
   api("us.ihmc:jinput:2.0.6-ihmc2")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:ihmc-commons:0.30.4")
   api("us.ihmc:log-tools:0.6.3")
   api("us.ihmc:euclid:0.16.2")
   api("us.ihmc:euclid-frame:0.16.2")
   api("us.ihmc:euclid-shape:0.16.2")
   api("us.ihmc:euclid-geometry:0.16.2")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
