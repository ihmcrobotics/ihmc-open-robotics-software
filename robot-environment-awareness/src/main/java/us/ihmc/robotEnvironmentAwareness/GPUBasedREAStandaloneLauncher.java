package us.ihmc.robotEnvironmentAwareness;

import java.net.URI;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.GPUPerceptionModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.GPUBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUBasedREAModule;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;

public class GPUBasedREAStandaloneLauncher extends ApplicationNoModule
{

   private GPUBasedEnvironmentAwarenessUI ui;
   private GPUBasedREAModule module;

   private ROS2Node ros2Node;
   private RosMainNode rosMainNode;

   private boolean ENABLE_UI = true;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      URI rosMasterURI = NetworkParameters.getROSURI();
      rosMainNode = new RosMainNode(rosMasterURI, "GPUPlanarRegionSubscriber");
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, PerceptionAPI.GPU_REA_NODE_NAME);

      SharedMemoryJavaFXMessager messager = new SharedMemoryJavaFXMessager(GPUPerceptionModuleAPI.API);
      if (ENABLE_UI)
         ui = GPUBasedEnvironmentAwarenessUI.createIntraprocessUI(messager, primaryStage);
      module = GPUBasedREAModule.createIntraprocess(messager, ros2Node, rosMainNode);

      rosMainNode.execute();
      if (ENABLE_UI)
         ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {

      super.stop();
      rosMainNode.shutdown();
      ros2Node.destroy();
      if (ENABLE_UI)
         ui.stop();
      module.stop();
      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
