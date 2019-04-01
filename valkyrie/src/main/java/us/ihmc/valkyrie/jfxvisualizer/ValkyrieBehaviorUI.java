package us.ihmc.valkyrie.jfxvisualizer;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.BehaviorTeleop;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieBehaviorUI extends Application
{
   private static final boolean launchBehaviorModule = false;

   private SharedMemoryJavaFXMessager messager;

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel drcRobotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, false);
      messager = new SharedMemoryJavaFXMessager(BehaviorUI.API.create());

      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_behavior_ui");
//      AtlasLowLevelMessenger robotLowLevelMessenger = new AtlasLowLevelMessenger(ros2Node, drcRobotModel.getSimpleRobotName());

      BehaviorTeleop teleop = BehaviorTeleop.createForUI(drcRobotModel, "localhost");

      ui = new BehaviorUI(primaryStage,
                          teleop,
                          drcRobotModel,
                          PubSubImplementation.FAST_RTPS);
      ui.show();

      messager.startMessager();

      if (launchBehaviorModule)
      {
         // launch behavior module
      }
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      messager.closeMessager();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
