package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.*;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class is a toolbox module that provides the ability to stream kinematics commands to the robot.
 * <p>
 * It's useful to understand the API of the IK streaming.
 * </p>
 * <p>
 * Typically extended for each robot, the final implementation contains a main to run as a standalone app.
 * </p>
 */
public class KinematicsStreamingToolboxModule extends ToolboxModule
{
   protected final KinematicsStreamingToolboxController controller;
   private ROS2PublisherBasics<WholeBodyTrajectoryMessage> trajectoryMessagePublisher;
   private ROS2PublisherBasics<WholeBodyStreamingMessage> streamingMessagePublisher;

   RobotConfigurationDataBasedUpdater robotStateUpdater = new RobotConfigurationDataBasedUpdater();

   public KinematicsStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      this(robotModel, KinematicsStreamingToolboxParameters.defaultParameters(), startYoVariableServer, pubSubImplementation);
   }

   public KinematicsStreamingToolboxModule(DRCRobotModel robotModel,
                                           KinematicsStreamingToolboxParameters parameters,
                                           boolean startYoVariableServer,
                                           PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(),
            robotModel.createFullRobotModel(),
            robotModel.getLogModelProvider(),
            startYoVariableServer,
            (int) (parameters.getToolboxUpdatePeriod() * 1000),
            pubSubImplementation);

      setTimeWithoutInputsBeforeGoingToSleep(parameters.getTimeThresholdForSleeping());
      controller = new KinematicsStreamingToolboxController(commandInputManager,
                                                            statusOutputManager,
                                                            parameters,
                                                            fullRobotModel,
                                                            robotModel,
                                                            yoGraphicsListRegistry,
                                                            registry);
      controller.setRobotStateUpdater(robotStateUpdater);
      controller.setCollisionModel(robotModel.getHumanoidRobotKinematicsCollisionModel());
      Map<String, Double> initialConfiguration = fromStandPrep(robotModel);
      if (initialConfiguration != null)
         controller.setInitialRobotConfigurationNamedMap(initialConfiguration);
      controller.setTrajectoryMessagePublisher(trajectoryMessagePublisher::publish);
      controller.setStreamingMessagePublisher(streamingMessagePublisher::publish);
      startYoVariableServer();
      if (yoVariableServer != null)
      {
         JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer);
         jvmStatisticsGenerator.start();
      }
   }

   private static Map<String, Double> fromStandPrep(DRCRobotModel robotModel)
   {
      WholeBodySetpointParameters standPrepParameters = robotModel.getHighLevelControllerParameters().getStandPrepParameters();
      if (standPrepParameters == null)
         return null;

      Map<String, Double> initialConfigurationMap = new HashMap<>();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         String jointName = joint.getName();
         initialConfigurationMap.put(jointName, standPrepParameters.getSetpoint(jointName));
      }
      return initialConfigurationMap;
   }

   @Override
   public void registerExtraPuSubs(ROS2NodeInterface ros2Node)
   {
      trajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(WholeBodyTrajectoryMessage.class, robotName));
      streamingMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(WholeBodyStreamingMessage.class, robotName));

      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      ros2Node.createSubscription(StateEstimatorAPI.getRobotConfigurationDataTopic(robotName), s ->
      {
         s.takeNextData(robotConfigurationData, null);
         robotStateUpdater.setRobotConfigurationData(robotConfigurationData);
      });

      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(CapturabilityBasedStatus.class, robotName), s ->
      {
         if (controller != null)
         {
            s.takeNextData(capturabilityBasedStatus, null);
            controller.updateCapturabilityBasedStatus(capturabilityBasedStatus);
         }
      });
   }

   @Override
   public ToolboxController getToolboxController()
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
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsStreamingToolboxInputCommand.class);
      commands.add(KinematicsStreamingToolboxConfigurationCommand.class);
      commands.add(KinematicsToolboxConfigurationCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(KinematicsToolboxOutputStatus.class);
      status.add(ControllerCrashNotificationPacket.class);
      return status;
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ToolboxAPIs.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ToolboxAPIs.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName).withInput();
   }

   public static ROS2Topic<ToolboxStateMessage> getInputStateTopic(String robotName)
   {
      return getInputTopic(robotName).withTypeName(ToolboxStateMessage.class);
   }

   public static ROS2Topic<KinematicsStreamingToolboxInputMessage> getInputCommandTopic(String robotName)
   {
      return getInputTopic(robotName).withTypeName(KinematicsStreamingToolboxInputMessage.class);
   }

   public static ROS2Topic<KinematicsStreamingToolboxConfigurationMessage> getInputStreamingConfigurationTopic(String robotName)
   {
      return ControllerAPI.getTopic(getInputTopic(robotName), KinematicsStreamingToolboxConfigurationMessage.class);
   }

   public static ROS2Topic<KinematicsToolboxConfigurationMessage> getInputToolboxConfigurationTopic(String robotName)
   {
      return ControllerAPI.getTopic(getInputTopic(robotName), KinematicsToolboxConfigurationMessage.class);
   }

   public static ROS2Topic<KinematicsToolboxOutputStatus> getOutputStatusTopic(String robotName)
   {
      return ControllerAPI.getTopic(getOutputTopic(robotName), KinematicsToolboxOutputStatus.class);
   }

   public static ROS2Topic<ControllerCrashNotificationPacket> getOutputCrashNotificationTopic(String robotName)
   {
      return ControllerAPI.getTopic(getOutputTopic(robotName), ControllerCrashNotificationPacket.class);
   }
}
