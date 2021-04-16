package us.ihmc.gdx.ui;

import controller_msgs.msg.dds.*;
import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.behaviors.tools.ThrottledRobotStateCallback;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.string.StringTools;

public class ImGuiGDXDirectControlPanel
{
   private static final String WINDOW_NAME = "Direct Control";
   private static final double MIN_PELVIS_HEIGHT = 0.52;
   private static final double MAX_PELVIS_HEIGHT = 0.90;
   private static final double PELVIS_HEIGHT_RANGE = MAX_PELVIS_HEIGHT - MIN_PELVIS_HEIGHT;
   private static final double MIN_CHEST_PITCH = Math.toRadians(-15.0);
   private static final double MAX_CHEST_PITCH = Math.toRadians(50.0);
   private static final double CHEST_PITCH_RANGE = MAX_CHEST_PITCH - MIN_CHEST_PITCH;
   private static final double SLIDER_RANGE = 100.0;
   private static final double ROBOT_DATA_EXPIRATION = 1.0;
   private final CommunicationHelper communicationHelper;
   private final double neckJointJointLimitLower;
   private final double neckJointRange;

   private final ThrottledRobotStateCallback throttledRobotStateCallback;
   private final RobotLowLevelMessenger robotLowLevelMessenger;
   private final RemoteSyncedRobotModel syncedRobotForHeightSlider;
   private final RemoteSyncedRobotModel syncedRobotForChestSlider;

   private final float[] stanceHeightSliderValue = new float[1];
   private final float[] leanForwardSliderValue = new float[1];
   private final float[] neckPitchSliderValue = new float[1];

   private final ImInt pumpPSI = new ImInt(1);
   private final String[] psiValues = new String[] {"1500", "2300", "2500", "2800"};

   public ImGuiGDXDirectControlPanel(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;
      String robotName = communicationHelper.getRobotModel().getSimpleRobotName();
      ROS2NodeInterface ros2Node = communicationHelper.getROS2Node();
      DRCRobotModel robotModel = communicationHelper.getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      syncedRobotForHeightSlider = communicationHelper.newSyncedRobot();
      syncedRobotForChestSlider = communicationHelper.newSyncedRobot();

      robotLowLevelMessenger = communicationHelper.newRobotLowLevelMessenger();

      if (robotLowLevelMessenger == null)
      {
         throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotName);
      }

      OneDoFJointBasics neckJoint = fullRobotModel.getNeckJoint(NeckJointName.PROXIMAL_NECK_PITCH);
      double neckJointLimitUpper = neckJoint.getJointLimitUpper();
      neckJointJointLimitLower = neckJoint.getJointLimitLower();
      neckJointRange = neckJointLimitUpper - neckJointJointLimitLower;

      throttledRobotStateCallback = new ThrottledRobotStateCallback(ros2Node, robotModel, 5.0, syncedRobot ->
      {
         double pelvisZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
         double midFeetZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
         double midFeetToPelvis = pelvisZ - midFeetZ;
         double heightInRange = midFeetToPelvis - MIN_PELVIS_HEIGHT;
         double newHeightSliderValue = SLIDER_RANGE * heightInRange / PELVIS_HEIGHT_RANGE;
         stanceHeightSliderValue[0] = (float) newHeightSliderValue;

         FrameYawPitchRoll chestFrame = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
         chestFrame.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
         double leanForwardValue = chestFrame.getPitch();
         double pitchInRange = leanForwardValue - MIN_CHEST_PITCH;
         double newChestSliderValue = SLIDER_RANGE * pitchInRange / CHEST_PITCH_RANGE;
         double flippedChestSliderValue = 100.0 - newChestSliderValue;
         leanForwardSliderValue[0] = (float) flippedChestSliderValue;

         double neckAngle = syncedRobot.getFullRobotModel().getNeckJoint(NeckJointName.PROXIMAL_NECK_PITCH).getQ();
         double angleInRange = neckAngle - neckJointJointLimitLower;
         double newNeckSliderValue = SLIDER_RANGE * angleInRange / neckJointRange;
         double flippedNeckSliderValue = 100.0 - newNeckSliderValue;
         neckPitchSliderValue[0] = (float) flippedNeckSliderValue;
      });
   }

   public void render()
   {
      ImGui.begin(WINDOW_NAME);

      if (ImGui.button("Home Pose"))
      {
         double trajectoryTime = 3.5;

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
      if (ImGui.button("Stand prep"))
      {
         robotLowLevelMessenger.sendStandRequest();
      }
      if (ImGui.button("Freeze"))
      {
         robotLowLevelMessenger.sendFreezeRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Shutdown"))
      {
         robotLowLevelMessenger.sendShutdownRequest();
      }
      if (ImGui.combo("PSI", pumpPSI, psiValues, psiValues.length))
      {
         sendPSIRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Resend PSI"))
      {
         sendPSIRequest();
      }

      if (imGuiSlider("Height", stanceHeightSliderValue))
      {
         if (syncedRobotForHeightSlider.getDataReceptionTimerSnapshot().isRunning(ROBOT_DATA_EXPIRATION))
         {
            syncedRobotForHeightSlider.update();
            double sliderValue = stanceHeightSliderValue[0];
            double pelvisZ = syncedRobotForHeightSlider.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
            double midFeetZ = syncedRobotForHeightSlider.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
            double desiredHeight = MIN_PELVIS_HEIGHT + PELVIS_HEIGHT_RANGE * sliderValue / SLIDER_RANGE;
            double desiredHeightInWorld = desiredHeight + midFeetZ;
            LogTools.info(StringTools.format3D("Commanding height trajectory. slider: {} desired: {} (pelvis - midFeetZ): {} in world: {}",
                                               sliderValue,
                                               desiredHeight,
                                               pelvisZ - midFeetZ,
                                               desiredHeightInWorld));
            PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
            message.getEuclideanTrajectory()
                   .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(2.0,
                                                                              new Point3D(0.0, 0.0, desiredHeightInWorld),
                                                                              ReferenceFrame.getWorldFrame()));
            long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
            message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
            message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
            message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
            message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
            communicationHelper.publishToController(message);
         }
      }
      if (imGuiSlider("Lean Forward", leanForwardSliderValue))
      {
         if (syncedRobotForChestSlider.getDataReceptionTimerSnapshot().isRunning(ROBOT_DATA_EXPIRATION))
         {
            syncedRobotForChestSlider.update();
            double sliderValue = 100.0 - leanForwardSliderValue[0];
            double desiredChestPitch = MIN_CHEST_PITCH + CHEST_PITCH_RANGE * sliderValue / SLIDER_RANGE;

            FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobotForChestSlider.getReferenceFrames().getChestFrame());
            frameChestYawPitchRoll.changeFrame(syncedRobotForChestSlider.getReferenceFrames().getPelvisZUpFrame());
            frameChestYawPitchRoll.setPitch(desiredChestPitch);
            frameChestYawPitchRoll.changeFrame(ReferenceFrame.getWorldFrame());

            LogTools.info(StringTools.format3D("Commanding chest pitch. slider: {} pitch: {}", sliderValue, desiredChestPitch));

            ChestTrajectoryMessage message = new ChestTrajectoryMessage();
            message.getSo3Trajectory()
                   .set(HumanoidMessageTools.createSO3TrajectoryMessage(2.0,
                                                                        frameChestYawPitchRoll,
                                                                        EuclidCoreTools.zeroVector3D,
                                                                        ReferenceFrame.getWorldFrame()));
            long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
            message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

            communicationHelper.publishToController(message);
         }
      }
      if (imGuiSlider("Neck Pitch", neckPitchSliderValue))
      {
         double percent = neckPitchSliderValue[0] / 100.0;
         percent = 1.0 - percent;
         MathTools.checkIntervalContains(percent, 0.0, 1.0);
         double jointAngle = neckJointJointLimitLower + percent * neckJointRange;
         LogTools.info("Commanding neck trajectory: slider: {} angle: {}", neckPitchSliderValue[0], jointAngle);
         communicationHelper.publishToController(HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[] {jointAngle}));
      }

      ImGui.end();
   }

   private boolean imGuiSlider(String label, float[] value)
   {
      float previousValue = value[0];
      ImGui.sliderFloat(label, value, 0.0f, 100.0f);
      float currentValue = value[0];
      return currentValue != previousValue;
   }

   private void sendPSIRequest()
   {
      robotLowLevelMessenger.setHydraulicPumpPSI(Integer.parseInt(psiValues[pumpPSI.get()]));
   }

   public void destroy()
   {
      throttledRobotStateCallback.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
