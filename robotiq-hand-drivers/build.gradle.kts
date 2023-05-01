plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.24"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
   id("us.ihmc.scs") version "0.4"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.martiansoftware:jsap:2.1")
   api("net.wimpi:jamod:1.2")
   api("commons-collections:commons-collections:3.2.1")

   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

app.entrypoint("RobotiqControlThread", "us.ihmc.robotiq.control.RobotiqControlThread")
