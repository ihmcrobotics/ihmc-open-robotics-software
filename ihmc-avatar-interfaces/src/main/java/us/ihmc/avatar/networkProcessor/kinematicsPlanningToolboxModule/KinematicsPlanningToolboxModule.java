package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;

public class KinematicsPlanningToolboxModule extends ToolboxModule
{
   private final KinematicsPlanningToolboxController kinematicsPlanningToolboxController;

   public KinematicsPlanningToolboxModule(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider,
                                          boolean startYoVariableServer)
         throws IOException
   {
      super(drcRobotModel.getSimpleRobotName(), fullRobotModelToLog, modelProvider, startYoVariableServer);
      kinematicsPlanningToolboxController = new KinematicsPlanningToolboxController(drcRobotModel, fullRobotModel, commandInputManager, statusOutputManager,
                                                                                    registry);
      commandInputManager.registerConversionHelper(new KinematicsPlanningToolboxCommandConverter(fullRobotModel));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {

   }

   @Override
   public ToolboxController getToolboxController()
   {
      return kinematicsPlanningToolboxController;
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
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(KinematicsPlanningToolboxOutputStatus.class);
      return status;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
