package us.ihmc.avatar.networkProcessor.walkingPreview;

import controller_msgs.RobotConfigurationData;
import toolbox_msgs.WalkingControllerPreviewOutputMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.walkingPreviewToolboxAPI.WalkingControllerPreviewInputCommand;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;
import java.util.Collections;
import java.util.List;

public class WalkingControllerPreviewToolboxModule extends ToolboxModule
{
   private final WalkingControllerPreviewToolboxController controller;

   public WalkingControllerPreviewToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer)
         throws IOException
   {
      this(robotModel, startYoVariableServer, null);
   }

   public WalkingControllerPreviewToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, AsyncROS2Node asyncROS2Node)
         throws IOException
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS, asyncROS2Node);
      setTimeWithoutInputsBeforeGoingToSleep(60.0);

      controller = new WalkingControllerPreviewToolboxController(robotModel, 0.02, commandInputManager, statusOutputManager, registry);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(ROS2Node ros2Node)
   {
      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);

      ros2Node.createSubscription(controllerOutputTopic.withType(RobotConfigurationData.class), s ->
      {
         if (controller != null)
            controller.updateRobotConfigurationData(s.read());
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
   public List<Class<? extends ROS2Message<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   public static List<Class<? extends ROS2Message<?>>> supportedStatus()
   {
      return Collections.singletonList(WalkingControllerPreviewOutputMessage.class);
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ToolboxAPIs.WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ToolboxAPIs.WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withInput();
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
