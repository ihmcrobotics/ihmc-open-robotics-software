package us.ihmc.robotEnvironmentAwareness;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

import java.nio.file.Paths;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.LiveMapModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.RealSenseREANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.ui.LiveMapUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.LiveMapModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.ros2.ROS2Node;

public class LiveMapStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultLiveMapModuleConfiguration.txt";

   private Messager messager;
   private LiveMapUI ui;
   private LiveMapModule module;
   private ROS2Node ros2Node;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(LiveMapModuleAPI.API);
      messager.startMessager();
      
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
      
      REANetworkProvider realSenseREANetworkProvider = new RealSenseREANetworkProvider(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS);
      LIDARBasedREAModule reaModule = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(Paths.get(System.getProperty("user.home"))
                                                                                                               .resolve("RealsenseREAConfiguration.txt")
                                                                                                               .toFile()),
                                                                                   realSenseREANetworkProvider, NetworkPorts.REA_MODULE2_UI_PORT);
      reaModule.setParametersForStereo();
      reaModule.loadConfigurationsFromFile();
      reaModule.start();
      
      ui = LiveMapUI.createIntraprocessUI(messager, primaryStage);
      module = LiveMapModule.createIntraprocess(ros2Node, messager);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();
      ros2Node.destroy();

      messager.closeMessager();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
