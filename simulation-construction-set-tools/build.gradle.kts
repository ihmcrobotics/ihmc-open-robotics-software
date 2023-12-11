plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
   id("us.ihmc.scs") version "0.4"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid-frame:0.21.0")
   api("us.ihmc:euclid-frame-shape:0.21.0")
   api("us.ihmc:euclid-shape:0.21.0")
   api("us.ihmc:simulation-construction-set:0.24.3")
   api("us.ihmc:scs2-definition:17-0.19.0")
   api("us.ihmc:scs2-simulation-construction-set:17-0.18.0")
   api("us.ihmc:ihmc-parameter-optimization:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("com.github.stephengold:Minie:7.6.0")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")

   api("us.ihmc:simulation-construction-set-test:0.24.3")
}
