package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.io.IOException;
import java.util.Collections;
import java.util.List;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.walkingPreviewToolboxAPI.WalkingControllerPreviewInputCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WalkingControllerPreviewToolboxModule extends ToolboxModule
{
   private final WalkingControllerPreviewToolboxController controller;

   public WalkingControllerPreviewToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
         throws IOException
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, pubSubImplementation);
      setTimeWithoutInputsBeforeGoingToSleep(60.0);

      controller = new WalkingControllerPreviewToolboxController(robotModel, 0.02, commandInputManager, statusOutputManager, yoGraphicsListRegistry, registry);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator, s -> {
         if (controller != null)
            controller.updateRobotConfigurationData(s.takeNextData());
      });
   }

   @Override
   public WalkingControllerPreviewToolboxController getToolboxController()
   {
      return controller;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   public static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      return Collections.singletonList(WalkingControllerPreviewInputCommand.class);
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      return Collections.singletonList(WalkingControllerPreviewOutputMessage.class);
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.INPUT);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }
}
