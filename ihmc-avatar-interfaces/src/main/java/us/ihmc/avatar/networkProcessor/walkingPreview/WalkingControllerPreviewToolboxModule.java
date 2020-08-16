package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.io.IOException;
import java.util.Collections;
import java.util.List;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.walkingPreviewToolboxAPI.WalkingControllerPreviewInputCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.registry.YoRegistry;

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
      ROS2Topic controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, RobotConfigurationData.class, controllerOutputTopic, s -> {
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
   public ROS2Topic getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic getOutputTopic(String robotName)
   {
      return ROS2Tools.WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic getInputTopic(String robotName)
   {
      return ROS2Tools.WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withInput();
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }
}
