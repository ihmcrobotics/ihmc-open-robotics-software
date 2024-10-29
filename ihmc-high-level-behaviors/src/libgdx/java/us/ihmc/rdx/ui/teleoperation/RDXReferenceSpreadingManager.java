package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.ReferenceSpreadingToolboxInputMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.referenceSpreading.ReferenceSpreadingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.referenceSpreadingToolboxAPI.ReferenceSpreadingToolboxInputCommand;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDX3DPanelHandWrenchIndicator;
import us.ihmc.rdx.ui.affordances.RDXArmControlMode;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;

public class RDXReferenceSpreadingManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final CommunicationHelper communicationHelper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXDesiredRobot desiredRobot;
   private final RDXTeleoperationParameters teleoperationParameters;

   private ROS2Topic<ReferenceSpreadingToolboxInputMessage> referenceSpreadingROSTopic;
   private ROS2Topic<ToolboxStateMessage> toolboxStatePublisher;
   int sequenceId = 0;

   private final TypedNotification<RobotSide> showWarningNotification = new TypedNotification<>();

   private final ImBoolean toolboxActive = new ImBoolean(false);

   ROS2ControllerHelper ros2ControllerHelper;

   RDXReferenceSpreadingManager(CommunicationHelper communicationHelper,
                                DRCRobotModel robotModel,
                                ROS2SyncedRobotModel syncedRobot,
                                RDXDesiredRobot desiredRobot,
                                RDXTeleoperationParameters teleoperationParameters)
   {
      this.communicationHelper = communicationHelper;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.teleoperationParameters = teleoperationParameters;

      referenceSpreadingROSTopic = ReferenceSpreadingToolboxModule.getInputToolboxInputTopic(syncedRobot.getRobotModel().getSimpleRobotName());
      toolboxStatePublisher = ReferenceSpreadingToolboxModule.getInputTopic(robotModel.getSimpleRobotName()).withTypeName(ToolboxStateMessage.class);
      ros2ControllerHelper = this.communicationHelper.getControllerHelper();
   }

   public void create(RDXBaseUI baseUI)
   {
   }

   public void update(boolean interactablesEnabled)
   {

   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Toolbox Active"), toolboxActive))
      {
         if(toolboxActive.get())
         {
            startToolbox();
         }
         else
         {
            stopToolbox();
         }
      }

      if (ImGui.checkbox(labels.get("Record"), toolboxActive))
      {
         if(toolboxActive.get())
         {
            startRecording();
         }
         else
         {
            stopRecording();
         }
      }
   }

   private void startToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
      toolboxStateMessage.setRequestLogging(true);
      ros2ControllerHelper.publish(toolboxStatePublisher, toolboxStateMessage);
      LogTools.info("Start RS");
   }

   private void stopToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.SLEEP);
      toolboxStateMessage.setRequestLogging(true);
      ros2ControllerHelper.publish(toolboxStatePublisher, toolboxStateMessage);
      LogTools.info("Start RS");
   }

   private void startRecording()
   {
      ReferenceSpreadingToolboxInputMessage message = new ReferenceSpreadingToolboxInputMessage();
      message.setState((byte) 0);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

   private void stopRecording()
   {
      ReferenceSpreadingToolboxInputMessage message = new ReferenceSpreadingToolboxInputMessage();
      message.setState((byte) 1);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

}
