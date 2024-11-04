package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import imgui.type.ImBoolean;
import toolbox_msgs.msg.dds.ReferenceSpreadingToolboxInputMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.referenceSpreading.ReferenceSpreadingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;

public class RDXReferenceSpreadingManager
{
   private final ImGuiUniqueLabelMap labelRS = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiUniqueLabelMap labelToolbox = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiUniqueLabelMap labelsRecord = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiUniqueLabelMap labelsPlayback = new ImGuiUniqueLabelMap(getClass());

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
   private final ImBoolean recordActive = new ImBoolean(false);
   private final ImBoolean referenceSpreadingActive = new ImBoolean(false);
   private final ImBoolean normalPlaybackActive = new ImBoolean(false);

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
      ImGui.text("Record: ");
      ImGui.sameLine();
      if (recordActive.get())
      {
         if (ImGui.button(labelsRecord.get("Stop")))
         {
            stopRecording();
            recordActive.set(false);
         }
      }
      else
      {
         if (ImGui.button(labelsRecord.get("Start")))
         {
            startRecording();
            recordActive.set(true);
         }
      }

      ImGui.text("Toolbox: ");
      ImGui.sameLine();
      if (toolboxActive.get())
      {
         if (ImGui.button(labelToolbox.get("Stop")))
         {
            stopToolbox();
            toolboxActive.set(false);
         }
      }
      else
      {
         if (ImGui.button(labelToolbox.get("Start")))
         {
            startToolbox();
            toolboxActive.set(true);
         }
      }

      if (toolboxActive.get())
      {
         ImGui.text("Reference Spreading: ");
         ImGui.sameLine();
         if (referenceSpreadingActive.get())
         {
            if (ImGui.button(labelRS.get("Stop")))
            {
               stopReferenceSpreading();
               referenceSpreadingActive.set(false);
            }
         }
         else
         {
            if (ImGui.button(labelRS.get("Start")))
            {
               startReferenceSpreading();
               referenceSpreadingActive.set(true);
            }
         }
         ImGui.text("Normal Playback: ");
         ImGui.sameLine();
         if (normalPlaybackActive.get())
         {
            if (ImGui.button(labelsPlayback.get("Stop")))
            {
               stopNormalPlayback();
               normalPlaybackActive.set(false);
            }
         }
         else
         {
            if (ImGui.button(labelsPlayback.get("Start")))
            {
               startNormalPlayback();
               normalPlaybackActive.set(true);
            }
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
      message.setState((byte) 1);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

   private void stopRecording()
   {
      ReferenceSpreadingToolboxInputMessage message = new ReferenceSpreadingToolboxInputMessage();
      message.setState((byte) 0);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

   private void startReferenceSpreading()
   {
      ReferenceSpreadingToolboxInputMessage message = new ReferenceSpreadingToolboxInputMessage();
      message.setState((byte) 2);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

   private void stopReferenceSpreading()
   {
      ReferenceSpreadingToolboxInputMessage message = new ReferenceSpreadingToolboxInputMessage();
      message.setState((byte) 0);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

   private void startNormalPlayback()
   {
      ReferenceSpreadingToolboxInputMessage message = new ReferenceSpreadingToolboxInputMessage();
      message.setState((byte) 3);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

   private void stopNormalPlayback()
   {
      ReferenceSpreadingToolboxInputMessage message = new ReferenceSpreadingToolboxInputMessage();
      message.setState((byte) 0);
      message.setSequenceId(sequenceId++);
      ros2ControllerHelper.publish(referenceSpreadingROSTopic, message);

      LogTools.info("Message: " + message);
   }

}
