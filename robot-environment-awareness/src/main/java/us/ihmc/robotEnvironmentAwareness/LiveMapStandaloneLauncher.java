package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.LiveMapModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.LiveMapUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LiveMapModule;
import us.ihmc.ros2.ROS2Node;

public class LiveMapStandaloneLauncher extends ApplicationNoModule
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultLiveMapModuleConfiguration.txt";

   private Messager messager;
   private LiveMapUI ui;
   private LiveMapModule module;
   private ROS2Node ros2Node;

   private final boolean launchUI;
   private final PubSubImplementation pubSubImplementation;

   public LiveMapStandaloneLauncher()
   {
      this(true, PubSubImplementation.FAST_RTPS);
   }

   public LiveMapStandaloneLauncher(boolean launchUIs, PubSubImplementation pubSubImplementation)
   {
      this.launchUI = launchUIs;
      this.pubSubImplementation = pubSubImplementation;
   }


   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(LiveMapModuleAPI.API);
      messager.startMessager();
      
      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, PerceptionAPI.REA_NODE_NAME);

      if (launchUI)
         ui = LiveMapUI.createIntraprocessUI(messager, primaryStage);
      else
         ui = null;

      module = LiveMapModule.createIntraprocess(ros2Node, messager, "atlas");

      if (launchUI)
         ui.show();

      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      if (ui != null)
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
