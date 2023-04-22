package us.ihmc.atlas.sensors;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.RealSenseREANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.Path;
import java.nio.file.Paths;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

public class AtlasRealsenseBasedREAStandaloneLauncher
{
   private static final Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
   private static final String directory = "/atlas/src/main/resources/";

   private static final String REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME = "atlasRealSenseREAModuleConfiguration.txt";

   private final boolean spawnUI;

   private LIDARBasedEnvironmentAwarenessUI ui;
   private LIDARBasedREAModule module;

   public AtlasRealsenseBasedREAStandaloneLauncher(boolean spawnUI)
   {
      this.spawnUI = spawnUI;
      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(() -> ExceptionTools.handle(this::setup, DefaultExceptionHandler.PRINT_STACKTRACE));
   }

   private void setup() throws Exception
   {
      Stage primaryStage = null;
      if (spawnUI)
         primaryStage = new Stage();
      Path realsenseREAConfigurationFilePath = Paths.get(rootPath.toString(), directory + REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, PerceptionAPI.REA_NODE_NAME);
      REANetworkProvider realSenseREANetworkProvider = new RealSenseREANetworkProvider(ros2Node, stereoInputTopic, stereoOutputTopic);

      if (spawnUI)
         ui = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage,
                                                                   NetworkPorts.REA_MODULE2_UI_PORT);
      module = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(realsenseREAConfigurationFilePath.toFile()),
                                                            realSenseREANetworkProvider,
                                                            NetworkPorts.REA_MODULE2_UI_PORT);
      module.setParametersForStereo();
      module.loadConfigurationsFromFile();

      if (spawnUI)
         ui.show();
      module.start();

      if (spawnUI)
         primaryStage.setOnCloseRequest(event -> stop());
   }

   public void stop()
   {
      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      new AtlasRealsenseBasedREAStandaloneLauncher(true);
   }
}
