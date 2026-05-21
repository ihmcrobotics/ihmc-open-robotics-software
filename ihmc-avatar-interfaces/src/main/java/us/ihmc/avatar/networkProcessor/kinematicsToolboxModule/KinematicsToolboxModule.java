package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.jros2.ROS2Message;

import controller_msgs.CapturabilityBasedStatus;
import controller_msgs.MultiContactBalanceStatus;
import controller_msgs.RobotConfigurationData;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxInputCollectionCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxOneDoFJointCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxPrivilegedConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxSupportRegionCommand;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.jros2.AsyncROS2Node;

import java.util.ArrayList;
import java.util.List;

public class KinematicsToolboxModule extends ToolboxModule
{
   private static final boolean DEFAULT_SETUP_INITIAL_CONFIGURATION = true;

   private final HumanoidKinematicsToolboxController kinematicsToolBoxController;
   private final RobotConfigurationDataBasedUpdater robotStateUpdater = new RobotConfigurationDataBasedUpdater();

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, AsyncROS2Node realtimeROS2Node)
   {
      this(robotModel, startYoVariableServer, DEFAULT_UPDATE_PERIOD_MILLISECONDS, DEFAULT_SETUP_INITIAL_CONFIGURATION, realtimeROS2Node);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer)
   {
      this(robotModel, startYoVariableServer, DEFAULT_UPDATE_PERIOD_MILLISECONDS);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel,
                                  boolean startYoVariableServer,
                                  int updatePeriodMilliseconds)
   {
      this(robotModel, startYoVariableServer, updatePeriodMilliseconds, DEFAULT_SETUP_INITIAL_CONFIGURATION);
   }

   public KinematicsToolboxModule(DRCRobotModel robotModel,
                                  boolean startYoVariableServer,
                                  int updatePeriodMilliseconds,
                                  boolean setupInitialConfiguration)
   {
      this(robotModel, startYoVariableServer, updatePeriodMilliseconds, setupInitialConfiguration, null);
   }

   private KinematicsToolboxModule(DRCRobotModel robotModel,
                                   boolean startYoVariableServer,
                                   int updatePeriodMilliseconds,
                                   boolean setupInitialConfiguration,
                                   AsyncROS2Node realtimeROS2Node)
   {
      super(robotModel.getSimpleRobotName(),
            robotModel.createFullRobotModel(),
            robotModel.getLogModelProvider(),
            startYoVariableServer,
            updatePeriodMilliseconds,
            realtimeROS2Node);
      kinematicsToolBoxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                            statusOutputManager,
                                                                            fullRobotModel,
                                                                            0.001,
                                                                            registry);
      graphicGroupDefinition.addChild(kinematicsToolBoxController.getSCS2YoGraphics());
      kinematicsToolBoxController.setDesiredRobotStateUpdater(robotStateUpdater);
      if (setupInitialConfiguration)
         kinematicsToolBoxController.setInitialRobotConfiguration(robotModel);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(fullRobotModel,
                                                                                         kinematicsToolBoxController.getDesiredReferenceFrames()));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(ROS2Node ros2Node)
   {
      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      ros2Node.createSubscription(StateEstimatorAPI.getRobotConfigurationDataTopic(robotName), reader ->
      {
         if (kinematicsToolBoxController != null)
         {
            reader.read(robotConfigurationData);
            robotStateUpdater.setRobotConfigurationData(robotConfigurationData);
         }
      });

      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(CapturabilityBasedStatus.class, robotName), reader ->
      {
         if (kinematicsToolBoxController != null)
         {
            reader.read(capturabilityBasedStatus);
            kinematicsToolBoxController.updateCapturabilityBasedStatus(capturabilityBasedStatus);
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
   public List<Class<? extends ROS2Message<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   public static List<Class<? extends ROS2Message<?>>> supportedStatus()
   {
      List<Class<? extends ROS2Message<?>>> status = new ArrayList<>();
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
      return ToolboxAPIs.KINEMATICS_TOOLBOX.appendedWith(robotName).appendedWith("output");
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ToolboxAPIs.KINEMATICS_TOOLBOX.appendedWith(robotName).appendedWith("input");
   }
}
