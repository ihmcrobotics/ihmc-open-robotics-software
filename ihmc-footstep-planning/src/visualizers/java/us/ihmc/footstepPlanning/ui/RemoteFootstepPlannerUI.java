package us.ihmc.footstepPlanning.ui;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.ApplicationNoModule;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

/**
 * This class provides a visualizer for the remote footstep planner found in the footstep planner toolbox.
 * It allows users to view the resulting plans calculated by the toolbox. It also allows the user to tune
 * the planner parameters, and request a new plan from the planning toolbox.
 */
public class RemoteFootstepPlannerUI extends ApplicationNoModule
{
   private SharedMemoryJavaFXMessager messager;
   private RemoteUIMessageConverter messageConverter;

   private FootstepPlannerUI ui;
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("height_map_heartbeat_node");
   private ROS2Heartbeat heightMapHeartbeat;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      messageConverter = RemoteUIMessageConverter.createConverter(messager, "");


      heightMapHeartbeat = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP);
      heightMapHeartbeat.setAlive(true);

      messager.startMessager();

      ui = FootstepPlannerUI.createUI(primaryStage, messager);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      heightMapHeartbeat.destroy();
      ros2Node.destroy();

      super.stop();

      messager.closeMessager();
      messageConverter.destroy();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
