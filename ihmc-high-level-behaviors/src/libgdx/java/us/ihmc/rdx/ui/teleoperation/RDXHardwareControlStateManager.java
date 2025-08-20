package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.EnableHPUCommandMessage;
import controller_msgs.msg.dds.EnableHPUStatusMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.MasterGainScaleControllerCommandMessage;
import controller_msgs.msg.dds.MasterGainScaleControllerStatusMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.Timer;

public class RDXHardwareControlStateManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final Timer hpuConnectedTimer = new Timer();
   private final Timer robotServoedConnectedTimer = new Timer();
   private boolean isRobotServoed = false;
   private HighLevelControllerName currentHighLevelState = null;

   public RDXHardwareControlStateManager(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;

      communicationHelper.subscribeToControllerViaVolatileCallback(MasterGainScaleControllerStatusMessage.class, message ->
      {
         robotServoedConnectedTimer.reset();
         isRobotServoed = message.getIsRobotServoed();
      });
      communicationHelper.subscribeToControllerViaVolatileCallback(MasterGainScaleControllerStatusMessage.class, message ->
      {
         robotServoedConnectedTimer.reset();
         isRobotServoed = message.getIsRobotServoed();
      });
      communicationHelper.subscribeToControllerViaVolatileCallback(HighLevelStateChangeStatusMessage.class, message ->
      {  // TODO: Create a HighLevelStateStatusMessage that is periodically published, so we can always know current state
         currentHighLevelState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      });
   }

   public void renderImGuiWidgets()
   {
      if (robotServoedConnectedTimer.isRunning(1.0))
      {
         if (isRobotServoed)
         {
            if (ImGui.button(labels.get("Unservo Slowly")))
            {
               MasterGainScaleControllerCommandMessage masterGainScaleControllerCommandMessage = new MasterGainScaleControllerCommandMessage();
               masterGainScaleControllerCommandMessage.setUnservoSlowly(true);
               communicationHelper.publishToController(masterGainScaleControllerCommandMessage);
            }
         }
         else
         {
            if (ImGui.button(labels.get("Servo Robot")))
            {
               MasterGainScaleControllerCommandMessage masterGainScaleControllerCommandMessage = new MasterGainScaleControllerCommandMessage();
               masterGainScaleControllerCommandMessage.setServoRobot(true);
               communicationHelper.publishToController(masterGainScaleControllerCommandMessage);
            }
         }
      }

      float widgetStartX = 98.0f;
      ImGui.text("Current Controller State: %s".formatted(currentHighLevelState == null ? "Unknown" : currentHighLevelState.name()));
      ImGui.separator();
      ImGui.text("Request:");
      ImGui.sameLine();
      ImGui.setCursorPosX(widgetStartX);
      if (ImGui.button(labels.get("Calibration")))
      {
         HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
         highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.CALIBRATION.toByte());
         communicationHelper.publishToController(highLevelStateMessage);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Freeze")))
      {
         RDXBaseUI.pushNotification("Commanding freeze...");
         sendFreezeRequest();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Stand Prep")))
      {
         RDXBaseUI.pushNotification("Commanding stand prep...");
         sendStandPrepRequest();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Stand Prep Transition")))
      {
         RDXBaseUI.pushNotification("Commanding stand prep transition...");
         sendStandPrepTransitionRequest();
      }
      ImGui.separator();
      ImGui.text("Command:");
      ImGui.sameLine();
      ImGui.setCursorPosX(widgetStartX);
      if (ImGui.button(labels.get("Home Pose")))
      {
         double trajectoryTime = 3.0;

         GoHomeMessage homeLeftArm = new GoHomeMessage();
         homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
         homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
         homeLeftArm.setTrajectoryTime(trajectoryTime);
         communicationHelper.publishToController(homeLeftArm);

         GoHomeMessage homeRightArm = new GoHomeMessage();
         homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
         homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
         homeRightArm.setTrajectoryTime(trajectoryTime);
         communicationHelper.publishToController(homeRightArm);

         GoHomeMessage homePelvis = new GoHomeMessage();
         homePelvis.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_PELVIS);
         homePelvis.setTrajectoryTime(trajectoryTime);
         communicationHelper.publishToController(homePelvis);

         GoHomeMessage homeChest = new GoHomeMessage();
         homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
         homeChest.setTrajectoryTime(trajectoryTime);

         RDXBaseUI.pushNotification("Commanding home pose...");
         communicationHelper.publishToController(homeChest);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop All Trajectories")))
      {
         RDXBaseUI.pushNotification("Commanding stop all trajectories...");
         StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
         communicationHelper.publishToController(stopAllTrajectoryMessage);
      }
   }

   public void sendStandPrepRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.STAND_PREP_STATE.toByte());
      communicationHelper.publishToController(highLevelStateMessage);
   }

   public void sendStandPrepTransitionRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.STAND_TRANSITION_STATE.toByte());
      communicationHelper.publishToController(highLevelStateMessage);
   }

   public void sendFreezeRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.FREEZE_STATE.toByte());
      communicationHelper.publishToController(highLevelStateMessage);
   }
}
