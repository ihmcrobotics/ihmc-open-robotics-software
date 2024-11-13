plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.4"
   id("us.ihmc.scs") version "0.4"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("net.wimpi:jamod:1.2")

   api("us.ihmc:ihmc-avatar-interfaces:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

app.entrypoint("RobotiqControlThread", "us.ihmc.robotiq.control.RobotiqControlThread")
