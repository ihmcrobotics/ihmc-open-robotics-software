package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import us.ihmc.jros2.ROS2Message;

import controller_msgs.RobotConfigurationData;
import controller_msgs.RobotDesiredConfigurationData;
import toolbox_msgs.ExternalForceEstimationOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI.ExternalForceEstimationToolboxConfigurationCommand;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.jros2.AsyncROS2Node;

import java.util.ArrayList;
import java.util.List;

public class ExternalForceEstimationToolboxModule extends ToolboxModule
{
   public static final int UPDATE_PERIOD_MILLIS = 60;
   private static final double defaultTimeWithoutInputsBeforeSleep = 60.0;

   private final ExternalForceEstimationToolboxController forceEstimationToolboxController;

   public ExternalForceEstimationToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, AsyncROS2Node ros2Node)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, UPDATE_PERIOD_MILLIS, ros2Node);
      this.forceEstimationToolboxController = new ExternalForceEstimationToolboxController(robotModel, fullRobotModel, commandInputManager, statusOutputManager, yoGraphicsListRegistry, UPDATE_PERIOD_MILLIS, registry);
      timeWithoutInputsBeforeGoingToSleep.set(defaultTimeWithoutInputsBeforeSleep);
      startYoVariableServer();
   }

   public ExternalForceEstimationToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, UPDATE_PERIOD_MILLIS);
      this.forceEstimationToolboxController = new ExternalForceEstimationToolboxController(robotModel, fullRobotModel, commandInputManager, statusOutputManager, yoGraphicsListRegistry, UPDATE_PERIOD_MILLIS, registry);
      timeWithoutInputsBeforeGoingToSleep.set(defaultTimeWithoutInputsBeforeSleep);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(ROS2Node ros2Node)
   {
      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);

      ros2Node.createSubscription(controllerOutputTopic.withType(RobotConfigurationData.class), s ->
      {
         if(forceEstimationToolboxController != null)
            forceEstimationToolboxController.updateRobotConfigurationData(s.read());
      });

      ros2Node.createSubscription(controllerOutputTopic.withType(RobotDesiredConfigurationData.class), s ->
      {
         if(forceEstimationToolboxController != null)
            forceEstimationToolboxController.updateRobotDesiredConfigurationData(s.read());
      });
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return forceEstimationToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return getSupportedCommands();
   }

   public static List<Class<? extends Command<?, ?>>> getSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(ExternalForceEstimationToolboxConfigurationCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends ROS2Message<?>>> createListOfSupportedStatus()
   {
      return getSupportedStatuses();
   }

   public static List<Class<? extends ROS2Message<?>>> getSupportedStatuses()
   {
      List<Class<? extends ROS2Message<?>>> status = new ArrayList<>();
      status.add(ExternalForceEstimationOutputStatus.class);
      return status;
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ToolboxAPIs.EXTERNAL_FORCE_ESTIMATION_TOOLBOX.appendedWith(robotName).appendedWith("output");
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ToolboxAPIs.EXTERNAL_FORCE_ESTIMATION_TOOLBOX.appendedWith(robotName).appendedWith("input");
   }
}
