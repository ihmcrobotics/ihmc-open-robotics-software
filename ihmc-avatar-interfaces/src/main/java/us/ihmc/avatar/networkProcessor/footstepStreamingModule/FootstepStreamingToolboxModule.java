package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxInputCommand;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;

/**
 * This class implements a toolbox module designed to stream footstep commands to the robot based on real-time VR user input.
 * It continuously estimates the robot's target footstep by tracking the user's stepping movements in VR.
 * The primary goal of this module is to provide a fast and responsive footstep estimate, enabling the robot to move in sync with the user.
 * Simultaneously, the target footstep is dynamically updated to reflect the user's actual stepping position,
 * ensuring accurate and adaptive robot locomotion.
 * <p>
 * The input typically comes from VR trackers strapped around the ankles of the user
 * </p>
 * <p>
 * The module is typically extended for each robot, the final implementation contains a main to run as a standalone app.
 * </p>
 */
public class FootstepStreamingToolboxModule extends ToolboxModule
{
   protected final FootstepStreamingToolboxController controller;

   public FootstepStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer)
   {
      this(robotModel, FootstepStreamingToolboxParameters.defaultParameters(), startYoVariableServer);
   }

   public FootstepStreamingToolboxModule(DRCRobotModel robotModel,
                                         FootstepStreamingToolboxParameters parameters,
                                         boolean startYoVariableServer)
   {
      super(robotModel.getSimpleRobotName(),
            robotModel.createFullRobotModel(),
            robotModel.getLogModelProvider(),
            startYoVariableServer,
            (int) (parameters.getToolboxUpdatePeriod() * 1000));

      setTimeWithoutInputsBeforeGoingToSleep(parameters.getTimeThresholdForSleeping());
      controller = new FootstepStreamingToolboxController(commandInputManager,
                                                          statusOutputManager,
                                                          parameters,
                                                          fullRobotModel,
                                                          robotModel,
                                                          yoGraphicsListRegistry,
                                                          registry);
      startYoVariableServer();
      if (yoVariableServer != null)
      {
         JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer);
         jvmStatisticsGenerator.start();
      }
   }

   @Override
   public void registerExtraPuSubs(ROS2Node ros2Node)
   {
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
      commands.add(FootstepStreamingToolboxInputCommand.class);
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
      status.add(FootstepStreamingToolboxOutputStatus.class);
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
      return ToolboxAPIs.FOOTSTEP_STREAMING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ToolboxAPIs.FOOTSTEP_STREAMING_TOOLBOX.withRobot(robotName).withInput();
   }

   public static ROS2Topic<ToolboxStateMessage> getInputStateTopic(String robotName)
   {
      return getInputTopic(robotName).withTypeName(ToolboxStateMessage.class);
   }

   public static ROS2Topic<FootstepStreamingToolboxInputMessage> getInputCommandTopic(String robotName)
   {
      return getInputTopic(robotName).withTypeName(FootstepStreamingToolboxInputMessage.class);
   }

   public static ROS2Topic<FootstepStreamingToolboxOutputStatus> getOutputStatusTopic(String robotName)
   {
      return getOutputTopic(robotName).withTypeName(FootstepStreamingToolboxOutputStatus.class);
   }
}
