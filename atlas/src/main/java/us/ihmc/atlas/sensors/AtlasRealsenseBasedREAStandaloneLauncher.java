package us.ihmc.atlas.sensors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.RealSenseREANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.Path;
import java.nio.file.Paths;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

public class AtlasRealsenseBasedREAStandaloneLauncher extends Application
{
   private static final Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
   private static final String directory = "/atlas/src/main/resources/";

   private static final String REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME = "atlasRealSenseREAModuleConfiguration.txt";

   private LIDARBasedEnvironmentAwarenessUI ui;
   private LIDARBasedREAModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Path realsenseREAConfigurationFilePath = Paths.get(rootPath.toString(), directory + REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
      REANetworkProvider realSenseREANetworkProvider = new RealSenseREANetworkProvider(ros2Node, stereoOutputTopic);

      ui = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage,
                                                                NetworkPorts.REA_MODULE2_UI_PORT);
      module = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(realsenseREAConfigurationFilePath.toFile()),
                                                            realSenseREANetworkProvider,
                                                            NetworkPorts.REA_MODULE2_UI_PORT);
      module.setParametersForStereo();
      module.loadConfigurationsFromFile();

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
