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
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.Timer;

public class RDXRobotLowLevelMessenger
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RobotLowLevelMessenger robotLowLevelMessenger;
   private final CommunicationHelper communicationHelper;
   private final Timer hpuConnectedTimer = new Timer();
   private final Timer robotServoedConnectedTimer = new Timer();
   private boolean hpuEnabled = false;
   private boolean isRobotServoed = false;
   private HighLevelControllerName currentHighLevelState = null;
   private HighLevelControllerName highLevelStateToRequest = HighLevelControllerName.DO_NOTHING_BEHAVIOR;

   public RDXRobotLowLevelMessenger(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;

      robotLowLevelMessenger = communicationHelper.getOrCreateRobotLowLevelMessenger();
      if (robotLowLevelMessenger == null)
      {
         String robotName = communicationHelper.getRobotModel().getSimpleRobotName();
         throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotName);
      }

      communicationHelper.subscribeToControllerViaVolatileCallback(EnableHPUStatusMessage.class, message ->
      {
         hpuConnectedTimer.reset();
         hpuEnabled = message.getHpuEnabled();
      });
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
      if (hpuConnectedTimer.isRunning(1.0))
      {
         if (hpuEnabled)
         {
            ImGui.text("HPU is enabled.");
         }
         else
         {
            if (ImGui.button(labels.get("Enable HPU")))
            {
               EnableHPUCommandMessage enableHPUCommandMessage = new EnableHPUCommandMessage();
               enableHPUCommandMessage.setEnableHpu(true);
               communicationHelper.publishToController(enableHPUCommandMessage);
            }
         }
      }
      else
      {
         ImGui.textColored(ImGuiTools.DARK_RED, "HPU variable not connected.");
      }

      if (robotServoedConnectedTimer.isRunning(1.0))
      {
         if (isRobotServoed)
         {
            if (ImGui.button(labels.get("Unservo slowly")))
            {
               MasterGainScaleControllerCommandMessage masterGainScaleControllerCommandMessage = new MasterGainScaleControllerCommandMessage();
               masterGainScaleControllerCommandMessage.setUnservoSlowly(true);
               communicationHelper.publishToController(masterGainScaleControllerCommandMessage);
            }
         }
         else
         {
            if (ImGui.button(labels.get("Servo robot")))
            {
               MasterGainScaleControllerCommandMessage masterGainScaleControllerCommandMessage = new MasterGainScaleControllerCommandMessage();
               masterGainScaleControllerCommandMessage.setServoRobot(true);
               communicationHelper.publishToController(masterGainScaleControllerCommandMessage);
            }
         }
      }
      else
      {
         ImGui.textColored(ImGuiTools.DARK_RED, "Robot servoed variable not connected.");
      }

      ImGui.text("Current controller state: %s".formatted(currentHighLevelState == null ? "Unknown" : currentHighLevelState.name()));
      ImGui.text("Request:");
      ImGui.sameLine();
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
      if (ImGui.button(labels.get("Stand prep")))
      {
         RDXBaseUI.pushNotification("Commanding stand prep...");
         sendStandRequest();
      }

      ImGui.text("Command:");
      ImGui.sameLine();
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

   public void sendStandRequest()
   {
      robotLowLevelMessenger.sendStandRequest();
   }

   public void sendFreezeRequest()
   {
      robotLowLevelMessenger.sendFreezeRequest();
   }
}
