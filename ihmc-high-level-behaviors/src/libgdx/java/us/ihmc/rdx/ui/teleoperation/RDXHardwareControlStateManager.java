package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.EStopMasterGainCommandMessage;
import controller_msgs.msg.dds.EStopMasterGainStatusMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXArmManager;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;

public class RDXHardwareControlStateManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final Timer estopMasterGainStatusTimer = new Timer();
   private final ImBoolean estop = new ImBoolean();
   private final ImDouble masterGain = new ImDouble();
   private HighLevelControllerName currentHighLevelState = null;

   public RDXHardwareControlStateManager(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;

      communicationHelper.subscribeToControllerViaVolatileCallback(EStopMasterGainStatusMessage.class, message ->
      {
         estopMasterGainStatusTimer.reset();
         estop.set(message.getEstop());
         masterGain.set(message.getMasterGain());
      });
      communicationHelper.subscribeToControllerViaVolatileCallback(HighLevelStateChangeStatusMessage.class, message ->
      {  // TODO: Create a HighLevelStateStatusMessage that is periodically published, so we can always know current state
         currentHighLevelState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      });
   }

   public void renderImGuiWidgets(DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames, RDXArmManager armManager, double maxPelvisHeight)
   {
      if (estopMasterGainStatusTimer.isRunning(1.0))
      {
         ImGui.pushStyleColor(ImGuiCol.CheckMark, ImGuiTools.DARK_RED);
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
         boolean changed = ImGui.checkbox(labels.get("SOFT-E-STOP"), estop);
         ImGui.sameLine();
         changed |= ImGuiTools.sliderDouble(labels.get("Master Gain"), masterGain, 0.0, 1.0);
         ImGui.popStyleColor(2);

         if (changed)
         {
            EStopMasterGainCommandMessage message = new EStopMasterGainCommandMessage();
            message.setEstop(estop.get());
            message.setMasterGain(masterGain.get());
            communicationHelper.publishToController(message);
         }
      }

      float widgetStartX = 98.0f;
      ImGui.text("Current Controller State: %s".formatted(currentHighLevelState == null ? "Unknown" : currentHighLevelState.name()));

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
      if (ImGui.button(labels.get("N-Pose")))
      {
         goNPose(robotModel, referenceFrames, armManager, maxPelvisHeight);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop All Trajectories")))
      {
         RDXBaseUI.pushNotification("Commanding stop all trajectories...");
         StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
         communicationHelper.publishToController(stopAllTrajectoryMessage);
      }
   }

   private void goNPose(DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames, RDXArmManager armManager, double maxPelvisHeight)
   {
      double trajectoryTime = 3.0;

      for (RobotSide side : RobotSide.values)
         armManager.executeArmAngles(side, PresetArmConfiguration.N_POSE, trajectoryTime);

      if (robotModel.getRobotVersion().hasHead())
      {
         NeckJointName[] neckJointNamesArray = robotModel.getJointMap().getNeckJointNames();
         double[] desiredNeckJointValues = new double[neckJointNamesArray.length];
         for (int i = 0; i < neckJointNamesArray.length; i++)
         {
            desiredNeckJointValues[i] = 0.0; // TODO make 0 robot agnostic
         }
         communicationHelper.publishToController(HumanoidMessageTools.createHeadJointspaceTaskspaceTrajectoryMessage(referenceFrames,
                                                                                                            neckJointNamesArray,
                                                                                                            desiredNeckJointValues,
                                                                                                            trajectoryTime));
      }

      FramePose3D pelvisPose = new FramePose3D(referenceFrames.getMidFeetZUpFrame());
      pelvisPose.getTranslation().addZ(maxPelvisHeight - 0.02);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      communicationHelper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, pelvisPose));

      GoHomeMessage homeChest = new GoHomeMessage();
      homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
      homeChest.setTrajectoryTime(trajectoryTime);
      communicationHelper.publishToController(homeChest);
      RDXBaseUI.pushNotification("Commanding N-pose...");
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
