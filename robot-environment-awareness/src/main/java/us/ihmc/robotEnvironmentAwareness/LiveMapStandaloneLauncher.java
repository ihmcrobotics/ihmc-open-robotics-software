package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.ui.LiveMapUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.LiveMapModule;
import us.ihmc.ros2.Ros2Node;

public class LiveMapStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultLiveMapModuleConfiguration.txt";

   private LiveMapUI ui;
   private LiveMapModule module;
   private Ros2Node ros2Node;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
      ui = LiveMapUI.createIntraprocessUI(primaryStage);
      module = LiveMapModule.createIntraprocess(ros2Node);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();
      ros2Node.destroy();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
