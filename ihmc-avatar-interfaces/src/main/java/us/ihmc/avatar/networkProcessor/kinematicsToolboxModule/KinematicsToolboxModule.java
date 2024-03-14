package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.MultiContactBalanceStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.*;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

import java.util.ArrayList;
import java.util.List;

public class KinematicsToolboxModule extends ToolboxModule
{
   private static final boolean DEFAULT_SETUP_INITIAL_CONFIGURATION = true;

   private final HumanoidKinematicsToolboxController kinematicsToolBoxController;
   private final RobotConfigurationDataBasedUpdater robotStateUpdater = new RobotConfigurationDataBasedUpdater();

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, RealtimeROS2Node realtimeROS2Node)
   {
      this(robotModel, startYoVariableServer, DEFAULT_UPDATE_PERIOD_MILLISECONDS, DEFAULT_SETUP_INITIAL_CONFIGURATION, realtimeROS2Node, null);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      this(robotModel, startYoVariableServer, DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel,
                                  boolean startYoVariableServer,
                                  int updatePeriodMilliseconds,
                                  PubSubImplementation pubSubImplementation)
   {
      this(robotModel, startYoVariableServer, updatePeriodMilliseconds, DEFAULT_SETUP_INITIAL_CONFIGURATION, pubSubImplementation);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel,
                                  boolean startYoVariableServer,
                                  int updatePeriodMilliseconds,
                                  boolean setupInitialConfiguration,
                                  PubSubImplementation pubSubImplementation)
   {
      this(robotModel, startYoVariableServer, updatePeriodMilliseconds, setupInitialConfiguration, null, pubSubImplementation);
   }

   private KinematicsToolboxModule(DRCRobotModel robotModel,
                                   boolean startYoVariableServer,
                                   int updatePeriodMilliseconds,
                                   boolean setupInitialConfiguration,
                                   RealtimeROS2Node realtimeROS2Node,
                                   PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(),
            robotModel.createFullRobotModel(),
            robotModel.getLogModelProvider(),
            startYoVariableServer,
            updatePeriodMilliseconds,
            realtimeROS2Node,
            pubSubImplementation);
      kinematicsToolBoxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                            statusOutputManager,
                                                                            fullRobotModel,
                                                                            robotModel,
                                                                            0.001,
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      kinematicsToolBoxController.setDesiredRobotStateUpdater(robotStateUpdater);
      if (setupInitialConfiguration)
         kinematicsToolBoxController.setInitialRobotConfiguration(robotModel);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(fullRobotModel,
                                                                                         kinematicsToolBoxController.getDesiredReferenceFrames()));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(ROS2NodeInterface ros2Node)
   {
      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      ros2Node.createSubscription(StateEstimatorAPI.getRobotConfigurationDataTopic(robotName), s ->
      {
         if (kinematicsToolBoxController != null)
         {
            s.takeNextData(robotConfigurationData, null);
            robotStateUpdater.setRobotConfigurationData(robotConfigurationData);
         }
      });

      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(CapturabilityBasedStatus.class, robotName), s ->
      {
         if (kinematicsToolBoxController != null)
         {
            s.takeNextData(capturabilityBasedStatus, null);
            kinematicsToolBoxController.updateCapturabilityBasedStatus(capturabilityBasedStatus);
         }
      });

      MultiContactBalanceStatus multiContactBalanceStatus = new MultiContactBalanceStatus();

      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(MultiContactBalanceStatus.class, robotName), s ->
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
      commands.add(KinematicsToolboxSupportRegionCommand.class);
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
      return ToolboxAPIs.KINEMATICS_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ToolboxAPIs.KINEMATICS_TOOLBOX.withRobot(robotName).withInput();
   }
}
