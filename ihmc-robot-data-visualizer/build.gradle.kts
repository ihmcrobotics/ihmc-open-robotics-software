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
   api("com.martiansoftware:jsap:2.1")
   api("net.sourceforge.jmatio:jmatio:1.0")

   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.9")
   api("us.ihmc:simulation-construction-set:0.25.1")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-robot-data-logger:0.29.8")
}

testDependencies {
    api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

app.entrypoint("SCSVisualizer", "us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer")
app.entrypoint("GUICaptureViewer", "us.ihmc.robotDataVisualizer.gui.GUICaptureViewer")
