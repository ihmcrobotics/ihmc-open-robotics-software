package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXRobotLowLevelMessenger
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RobotLowLevelMessenger robotLowLevelMessenger;
   private final CommunicationHelper communicationHelper;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final ImInt pumpPSI = new ImInt(1);
   private final String[] psiValues = new String[] {"1500", "2300", "2500", "2800"};
   private final ROS2SyncedRobotModel syncedRobot;

   public RDXRobotLowLevelMessenger(ROS2SyncedRobotModel syncedRobot, CommunicationHelper communicationHelper, RDXTeleoperationParameters teleoperationParameters)
   {
      this.syncedRobot = syncedRobot;
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
         RDXBaseUI.pushNotification("Commanding freeze...");
         sendFreezeRequest();
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Stand prep")))
      {
         RDXBaseUI.pushNotification("Commanding stand prep...");
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

         RDXBaseUI.pushNotification("Commanding home pose...");
         communicationHelper.publishToController(homeChest);
      }

      if (ImGui.button(labels.get("Stop All Trajectories")))
      {
         RDXBaseUI.pushNotification("Commanding stop all trajectories...");
         StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
         communicationHelper.publishToController(stopAllTrajectoryMessage);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop All Wrench Trajectories")))
      {
         RDXBaseUI.pushNotification("Commanding stop all wrench trajectories...");

         double trajectoryDuration = 1.0;
         ReferenceFrame forceFrame = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getChestFrame).getReferenceFrame();

         for (RobotSide robotSide : RobotSide.values())
         {
            HandWrenchTrajectoryMessage wrenchTrajectoryMessage = new HandWrenchTrajectoryMessage();
            wrenchTrajectoryMessage.setRobotSide(robotSide.toByte());
            wrenchTrajectoryMessage.setForceExecution(true);
            WrenchTrajectoryMessage wrenchTrajectory = wrenchTrajectoryMessage.getWrenchTrajectory();

            WrenchTrajectoryPointMessage trajectoryPoint = HumanoidMessageTools.createWrenchTrajectoryPointMessage(0.0, new Vector3D(), new Vector3D());

            wrenchTrajectory.getWrenchTrajectoryPoints().add().set(trajectoryPoint);
            trajectoryPoint.setTime(trajectoryDuration);
            wrenchTrajectory.getWrenchTrajectoryPoints().add().set(trajectoryPoint);
            wrenchTrajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(forceFrame));

            wrenchTrajectory.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

            communicationHelper.publishToController(wrenchTrajectoryMessage);
         }
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
