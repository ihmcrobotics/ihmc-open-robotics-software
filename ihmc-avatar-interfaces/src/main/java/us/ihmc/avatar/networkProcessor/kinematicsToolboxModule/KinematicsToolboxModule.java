package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.MultiContactBalanceStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxContactStateCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxInputCollectionCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxOneDoFJointCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxPrivilegedConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

public class KinematicsToolboxModule extends ToolboxModule
{
   private final HumanoidKinematicsToolboxController kinematicsToolBoxController;

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      this(robotModel, startYoVariableServer, DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, int updatePeriodMilliseconds,
                                  PubSubImplementation pubSubImplementation)
   {
      this(robotModel, startYoVariableServer, updatePeriodMilliseconds, true, pubSubImplementation);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, int updatePeriodMilliseconds, boolean setupInitialConfiguration,
                                  PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer,
            updatePeriodMilliseconds, pubSubImplementation);
      kinematicsToolBoxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                            statusOutputManager,
                                                                            fullRobotModel,
                                                                            robotModel,
                                                                            0.001, // Note that the gains of the solver depend on this dt, it shouldn't change based on the actual thread scheduled period.
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      if (setupInitialConfiguration)
         kinematicsToolBoxController.setInitialRobotConfiguration(robotModel);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(fullRobotModel));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);

      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, RobotConfigurationData.class, controllerOutputTopic, s ->
      {
         if (kinematicsToolBoxController != null)
         {
            s.takeNextData(robotConfigurationData, null);
            kinematicsToolBoxController.updateRobotConfigurationData(robotConfigurationData);
         }
      });

      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, CapturabilityBasedStatus.class, controllerOutputTopic, s ->
      {
         if (kinematicsToolBoxController != null)
         {
            s.takeNextData(capturabilityBasedStatus, null);
            kinematicsToolBoxController.updateCapturabilityBasedStatus(capturabilityBasedStatus);
         }
      });

      MultiContactBalanceStatus multiContactBalanceStatus = new MultiContactBalanceStatus();

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, MultiContactBalanceStatus.class, controllerOutputTopic, s ->
      {
         if (kinematicsToolBoxController != null)
         {
            s.takeNextData(multiContactBalanceStatus, null);
            kinematicsToolBoxController.updateMultiContactBalanceStatus(multiContactBalanceStatus);
         }
      });
   }

   /**
    * This method defines the input API for this toolbox. You can find the corresponding messages to
    * these commands that can be sent over the network.
    * <p>
    * Do not forget that this toolbox will ignore any message with a destination different from
    * {@value KinematicsToolboxModule#PACKET_DESTINATION}.
    * </p>
    */
   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   public static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsToolboxCenterOfMassCommand.class);
      commands.add(KinematicsToolboxRigidBodyCommand.class);
      commands.add(KinematicsToolboxOneDoFJointCommand.class);
      commands.add(KinematicsToolboxConfigurationCommand.class);
      commands.add(KinematicsToolboxContactStateCommand.class);
      commands.add(KinematicsToolboxPrivilegedConfigurationCommand.class);
      commands.add(KinematicsToolboxInputCollectionCommand.class);
      commands.add(HumanoidKinematicsToolboxConfigurationCommand.class);
      return commands;
   }

   /**
    * This method defines the output API for this toolbox. The message that this toolbox sends are
    * directed to the source the of the input messages.
    */
   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(KinematicsToolboxOutputStatus.class);
      return status;
   }

   @Override
   public KinematicsToolboxController getToolboxController()
   {
      return kinematicsToolBoxController;
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ROS2Tools.KINEMATICS_TOOLBOX.withRobot(robotName).withInput();
   }
}
