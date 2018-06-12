package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import controller_msgs.msg.dds.CapturabilityBasedStatusPubSubType;
import controller_msgs.msg.dds.HeightQuadTreeMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools;
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

   public HeightQuadTreeToolboxModule(FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider) throws IOException
   {
      super(desiredFullRobotModel, modelProvider, false, 50);

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
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, new RobotConfigurationDataPubSubType(), "/ihmc/robot_configuration_data",
                                           s -> controller.receivedPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, new CapturabilityBasedStatusPubSubType(), "/ihmc/capturability_based_status",
                                           s -> controller.receivedPacket(s.takeNextData()));
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
      return new MessageTopicNameGenerator()
      {
         private final String prefix = TOOLBOX_ROS_TOPIC_PREFIX + "/height_quad_tree/input";

         @Override
         public String generateTopicName(Class<? extends Settable<?>> messageType)
         {
            return ROS2Tools.appendTypeToTopicName(prefix, messageType);
         }
      };
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return new MessageTopicNameGenerator()
      {
         private final String prefix = TOOLBOX_ROS_TOPIC_PREFIX + "/height_quad_tree/output";

         @Override
         public String generateTopicName(Class<? extends Settable<?>> messageType)
         {
            return ROS2Tools.appendTypeToTopicName(prefix, messageType);
         }
      };
   }
}
