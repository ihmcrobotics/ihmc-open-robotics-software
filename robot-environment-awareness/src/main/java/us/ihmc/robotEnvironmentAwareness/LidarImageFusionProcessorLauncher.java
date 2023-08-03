package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionProcessorCommunicationModule;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionProcessorUI;
import us.ihmc.ros2.ROS2Node;

public class LidarImageFusionProcessorLauncher extends ApplicationNoModule
{
   private SharedMemoryJavaFXMessager messager;

   private LidarImageFusionProcessorUI ui;
   private LidarImageFusionProcessorCommunicationModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, PerceptionAPI.STEREO_REA_NODE_NAME);
      //ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_lidar_image_fusion_ui");

      messager = new SharedMemoryJavaFXMessager(LidarImageFusionAPI.API);
      messager.startMessager();

      ui = LidarImageFusionProcessorUI.creatIntraprocessUI(ros2Node, messager, primaryStage);
      module = LidarImageFusionProcessorCommunicationModule.createIntraprocessModule(ros2Node, messager);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      messager.closeMessager();

      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}