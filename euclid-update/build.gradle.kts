plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.21.0")
}

geometryDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:euclid-geometry:0.21.0")
}

frameDependencies {
   api(ihmc.sourceSetProject("geometry"))
   api("us.ihmc:euclid-frame:0.21.0")
}

frameShapeDependencies {
   api(ihmc.sourceSetProject("frame"))
   api("us.ihmc:euclid-frame:0.21.0")
}

testDependencies {
   api(ihmc.sourceSetProject("frame-shape"))

}
