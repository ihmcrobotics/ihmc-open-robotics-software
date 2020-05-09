package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxOutputConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;

public class KinematicsStreamingToolboxModule extends ToolboxModule
{
   private static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 5;

   protected final KinematicsStreamingToolboxController controller;
   private IHMCRealtimeROS2Publisher<WholeBodyTrajectoryMessage> outputPublisher;

   public KinematicsStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);

      setTimeWithoutInputsBeforeGoingToSleep(3.0);
      controller = new KinematicsStreamingToolboxController(commandInputManager,
                                                            statusOutputManager,
                                                            fullRobotModel,
                                                            robotModel,
                                                            robotModel.getControllerDT(),
                                                            Conversions.millisecondsToSeconds(updatePeriodMilliseconds),
                                                            yoGraphicsListRegistry,
                                                            registry);
      controller.setCollisionModel(robotModel.getHumanoidRobotKinematicsCollisionModel());
      Map<String, Double> initialConfiguration = fromStandPrep(robotModel);
      if (initialConfiguration != null)
         controller.setInitialRobotConfigurationNamedMap(initialConfiguration);
      controller.setOutputPublisher(outputPublisher::publish);
      commandInputManager.registerConversionHelper(new KinematicsStreamingToolboxCommandConverter(fullRobotModel));
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
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Topic controllerInputTopic = ROS2Tools.getControllerInputTopic(robotName);
      ROS2Topic controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);

      outputPublisher = ROS2Tools.createPublisherTypeNamed(realtimeRos2Node, WholeBodyTrajectoryMessage.class, controllerInputTopic);

      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, RobotConfigurationData.class, controllerOutputTopic, s ->
      {
         if (controller != null)
         {
            s.takeNextData(robotConfigurationData, null);
            controller.updateRobotConfigurationData(robotConfigurationData);
         }
      });

      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, CapturabilityBasedStatus.class, controllerOutputTopic, s ->
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
      commands.add(KinematicsStreamingToolboxOutputConfigurationCommand.class);
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
   public ROS2Topic getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic getOutputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic getInputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName).withInput();
   }
}
