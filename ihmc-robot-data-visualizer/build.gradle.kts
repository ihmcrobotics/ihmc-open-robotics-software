plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
   id("us.ihmc.scs") version "0.4"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("net.sf.trove4j:trove4j:3.0.3")
   api("com.martiansoftware:jsap:2.1")
   api("net.sourceforge.jmatio:jmatio:1.0")

   api("us.ihmc:euclid:0.17.0")
   api("us.ihmc:ihmc-yovariables:0.9.11")
   api("us.ihmc:ihmc-video-codecs:2.1.6")
   api("us.ihmc:simulation-construction-set:0.21.13")
   api("us.ihmc:ihmc-javafx-toolkit:0.20.0")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
   api("us.ihmc:ihmc-robot-description:0.21.3")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-robot-data-logger:0.20.9") {
      exclude(group = "org.junit.jupiter", module = "junit-jupiter-api")
      exclude(group = "org.junit.jupiter", module = "junit-jupiter-engine")
      exclude(group = "org.junit.platform", module = "junit-platform-commons")
      exclude(group = "org.junit.platform", module = "junit-platform-launcher")
   }
}

testDependencies {
    api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

app.entrypoint("SCSVisualizer", "us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer")
app.entrypoint("GUICaptureViewer", "us.ihmc.robotDataVisualizer.gui.GUICaptureViewer")
