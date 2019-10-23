package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HeightQuadTreeMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.HeightQuadTreeToolboxRequestCommand;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.LidarScanCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;

public class HeightQuadTreeToolboxModule extends ToolboxModule
{
   private final HeightQuadTreeToolboxController controller;

   public HeightQuadTreeToolboxModule(String robotName, FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider) throws IOException
   {
      super(robotName, desiredFullRobotModel, modelProvider, false, 50);

      controller = new HeightQuadTreeToolboxController(fullRobotModel, commandInputManager, statusOutputManager, registry);
      setTimeWithoutInputsBeforeGoingToSleep(3.0);
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return controller;
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if (controller != null)
            controller.receivedPacket(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s ->
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
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_QUADTREE_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_QUADTREE_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
