package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import perception_msgs.msg.dds.HeightQuadTreeMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.HeightQuadTreeToolboxRequestCommand;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.LidarScanCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;

public class HeightQuadTreeToolboxModule extends ToolboxModule
{
   private final HeightQuadTreeToolboxController controller;

   public HeightQuadTreeToolboxModule(String robotName, FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider,
                                      PubSubImplementation pubSubImplementation)
   {
      super(robotName, desiredFullRobotModel, modelProvider, false, 50, pubSubImplementation);

      controller = new HeightQuadTreeToolboxController(fullRobotModel, commandInputManager, statusOutputManager, registry);
      setTimeWithoutInputsBeforeGoingToSleep(3.0);
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return controller;
   }

   @Override
   public void registerExtraPuSubs(RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, RobotConfigurationData.class, controllerOutputTopic, s ->
      {
         if (controller != null)
            controller.receivedPacket(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, CapturabilityBasedStatus.class, controllerOutputTopic, s ->
      {
         if (controller != null)
            controller.receivedPacket(s.takeNextData());
      });
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(HeightQuadTreeToolboxRequestCommand.class);
      commands.add(LidarScanCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return Collections.singletonList(HeightQuadTreeMessage.class);
   }

   @Override
   public Set<Class<? extends Command<?, ?>>> silentCommands()
   {
      return Collections.singleton(LidarScanCommand.class);
   }

   @Override
   public Set<Class<? extends Settable<?>>> filterExceptions()
   {
      return Collections.singleton(LidarScanMessage.class);
   }

   @Override
   public ROS2Topic getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic getOutputTopic(String robotName)
   {
      return ROS2Tools.HEIGHT_QUADTREE_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic getInputTopic(String robotName)
   {
      return ROS2Tools.HEIGHT_QUADTREE_TOOLBOX.withRobot(robotName).withInput();
   }
}
