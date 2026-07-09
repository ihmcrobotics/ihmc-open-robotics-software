plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
   id("us.ihmc.scs") version "0.4"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:simulation-construction-set:0.25.4")
   api("us.ihmc:scs2-simulation-construction-set:17-0.32.3")
   api("us.ihmc:ihmc-parameter-optimization:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("com.github.stephengold:Minie:7.6.0")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")

   api("us.ihmc:simulation-construction-set-test:0.25.4")
}
