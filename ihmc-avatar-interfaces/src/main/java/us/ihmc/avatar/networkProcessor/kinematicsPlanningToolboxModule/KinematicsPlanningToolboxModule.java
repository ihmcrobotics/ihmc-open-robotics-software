package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import toolbox_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class KinematicsPlanningToolboxModule extends ToolboxModule
{
   private final KinematicsPlanningToolboxController kinematicsPlanningToolboxController;

   public KinematicsPlanningToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation) throws IOException
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);
      kinematicsPlanningToolboxController = new KinematicsPlanningToolboxController(robotModel,
                                                                                    fullRobotModel,
                                                                                    commandInputManager,
                                                                                    statusOutputManager,
                                                                                    yoGraphicsListRegistry,
                                                                                    registry);
      commandInputManager.registerConversionHelper(new KinematicsPlanningToolboxCommandConverter(fullRobotModel, kinematicsPlanningToolboxController.getDesiredReferenceFrames()));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(ROS2NodeInterface ros2Node)
   {
      ROS2Topic controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerOutputTopic, s ->
      {
         if (kinematicsPlanningToolboxController != null)
            kinematicsPlanningToolboxController.updateRobotConfigurationData(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, CapturabilityBasedStatus.class, controllerOutputTopic, s ->
      {
         if (kinematicsPlanningToolboxController != null)
            kinematicsPlanningToolboxController.updateCapturabilityBasedStatus(s.takeNextData());
      });
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return kinematicsPlanningToolboxController;
   }

   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(KinematicsPlanningToolboxOutputStatus.class);
      status.add(KinematicsToolboxOutputStatus.class);
      return status;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsToolboxConfigurationCommand.class);
      commands.add(KinematicsPlanningToolboxCenterOfMassCommand.class);
      commands.add(KinematicsPlanningToolboxRigidBodyCommand.class);
      commands.add(KinematicsPlanningToolboxInputCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   @Override
   public ROS2Topic getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic getOutputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_PLANNING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic getInputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_PLANNING_TOOLBOX.withRobot(robotName).withInput();
   }
}
