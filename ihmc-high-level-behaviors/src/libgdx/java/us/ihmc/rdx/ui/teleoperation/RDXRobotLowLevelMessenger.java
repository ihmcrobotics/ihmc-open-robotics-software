package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXRobotLowLevelMessenger
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RobotLowLevelMessenger robotLowLevelMessenger;
   private final CommunicationHelper communicationHelper;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final ImInt pumpPSI = new ImInt(1);
   private final String[] psiValues = new String[] {"1500", "2300", "2500", "2800"};

   public RDXRobotLowLevelMessenger(CommunicationHelper communicationHelper, RDXTeleoperationParameters teleoperationParameters)
   {
      this.communicationHelper = communicationHelper;
      this.teleoperationParameters = teleoperationParameters;
      robotLowLevelMessenger = communicationHelper.getOrCreateRobotLowLevelMessenger();

      if (robotLowLevelMessenger == null)
      {
         String robotName = communicationHelper.getRobotModel().getSimpleRobotName();
         throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotName);
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button(labels.get("Freeze")))
      {
         sendFreezeRequest();
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Stand prep")))
      {
         sendStandRequest();
      }

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
         communicationHelper.publishToController(homeChest);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop All Trajectories")))
      {
         StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
         communicationHelper.publishToController(stopAllTrajectoryMessage);
      }

      if (teleoperationParameters.getPSIAdjustable())
      {
         if (ImGui.combo("PSI", pumpPSI, psiValues, psiValues.length))
         {
            sendPSIRequest();
         }
         ImGui.sameLine();
         if (ImGui.button("Resend PSI"))
         {
            sendPSIRequest();
         }
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

   private void sendPSIRequest()
   {
      robotLowLevelMessenger.setHydraulicPumpPSI(Integer.parseInt(psiValues[pumpPSI.get()]));
   }
}
