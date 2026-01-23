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
   api("us.ihmc:simulation-construction-set-group:0.25.3") // SCS1
   api("us.ihmc:ihmc-whole-body-controller:source")
}

testDependencies {
   api("us.ihmc:ihmc-communication-test:source")
}
