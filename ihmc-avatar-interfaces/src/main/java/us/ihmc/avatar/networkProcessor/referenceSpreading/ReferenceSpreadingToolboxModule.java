package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import toolbox_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI.ExternalForceEstimationToolboxConfigurationCommand;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;

public class ReferenceSpreadingToolboxModule extends ToolboxModule
{
   public static final int UPDATE_PERIOD_MILLIS = 60;
   private static final double defaultTimeWithoutInputsBeforeSleep = 60.0;

   private final ReferenceSpreadingToolboxController referenceSpreadingToolboxController;

   private ROS2PublisherBasics<HandHybridJointspaceTaskspaceTrajectoryMessage> trajectoryMessagePublisher;

   public ReferenceSpreadingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      this(robotModel, KinematicsStreamingToolboxParameters.defaultParameters(), startYoVariableServer, pubSubImplementation);
   }

   public ReferenceSpreadingToolboxModule(DRCRobotModel robotModel,
                                           KinematicsStreamingToolboxParameters parameters,
                                           boolean startYoVariableServer,
                                           PubSubImplementation pubSubImplementation){
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, UPDATE_PERIOD_MILLIS, pubSubImplementation);
      this.referenceSpreadingToolboxController = new ReferenceSpreadingToolboxController(robotModel, fullRobotModel, commandInputManager, statusOutputManager, yoGraphicsListRegistry, UPDATE_PERIOD_MILLIS, registry);
      timeWithoutInputsBeforeGoingToSleep.set(defaultTimeWithoutInputsBeforeSleep);

      referenceSpreadingToolboxController.setTrajectoryMessagePublisher(trajectoryMessagePublisher::publish);

      startYoVariableServer();
   }

   @Override
   public void receivedPacket(ToolboxStateMessage message)
   {
      super.receivedPacket(message);

      ToolboxState requestedState = ToolboxState.fromByte(message.getRequestedToolboxState());
      if (requestedState == null)
      {
         return;
      }

      if (requestedState == ToolboxState.REINITIALIZE)
      {
         referenceSpreadingToolboxController.resetToInitialState();
      }
   }

   @Override
   public void registerExtraPuSubs(ROS2NodeInterface ros2Node)
   {
      trajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(HandHybridJointspaceTaskspaceTrajectoryMessage.class, robotName));

      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);

      ros2Node.createSubscription(controllerOutputTopic.withTypeName(RobotConfigurationData.class), s ->
      {
         if(referenceSpreadingToolboxController != null)
            referenceSpreadingToolboxController.updateRobotConfigurationData(s.takeNextData());
      });
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return referenceSpreadingToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return getSupportedCommands();
   }

//todo: change to ReferenceSpreadingConfigurationCommand
   public static List<Class<? extends Command<?, ?>>> getSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(ExternalForceEstimationToolboxConfigurationCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return getSupportedStatuses();
   }

//   todo: change to ReferenceSpreadingOutputStatus
   public static List<Class<? extends Settable<?>>> getSupportedStatuses()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
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
      return ToolboxAPIs.REFERENCE_SPREADING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ToolboxAPIs.REFERENCE_SPREADING_TOOLBOX.withRobot(robotName).withInput();
   }
}
