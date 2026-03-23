package us.ihmc.rdx.ui.interactable;

import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;

public class RDXNeckPitchSlider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper ros2ControllerHelper;
   private String sliderName;
   private final ImInt sliderValue = new ImInt();
   private double trajectoryTimeSeconds;
   private int minPitchDeg = -30;
   private int maxPitchDeg = 30;
   private final Throttler sendThrottler = new Throttler();
   private final double sendPeriod = UnitConversions.hertzToSeconds(5.0);
   private NeckJointName[] neckJointNamesArray;
   private boolean[] neckJointIsPitch;
   private boolean hasPitchJoint;

   public RDXNeckPitchSlider(ROS2SyncedRobotModel syncedRobot,
                             ROS2ControllerHelper ros2ControllerHelper,
                             RDXTeleoperationParameters teleoperationParameters)
   {
      initialize(syncedRobot, ros2ControllerHelper, teleoperationParameters.getTrajectoryTime());
   }

   public RDXNeckPitchSlider(ROS2SyncedRobotModel syncedRobot,
                             ROS2ControllerHelper ros2ControllerHelper)
   {
      initialize(syncedRobot, ros2ControllerHelper, 1.0);
   }

   private void initialize(ROS2SyncedRobotModel syncedRobot,
                           ROS2ControllerHelper ros2ControllerHelper,
                           double trajectoryTimeSeconds)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.trajectoryTimeSeconds = trajectoryTimeSeconds;
      sliderName = "Neck pitch";

      if (syncedRobot.getRobotModel().getRobotVersion().hasHead())
      {
         neckJointNamesArray = syncedRobot.getRobotModel().getJointMap().getNeckJointNames();
         neckJointIsPitch = new boolean[neckJointNamesArray.length];
         boolean pitchJointFound = false;
         for (int i = 0; i < neckJointNamesArray.length; i++)
         {
            neckJointIsPitch[i] = neckJointNamesArray[i] == NeckJointName.DISTAL_NECK_PITCH
                                  || neckJointNamesArray[i] == NeckJointName.PROXIMAL_NECK_PITCH;
            pitchJointFound |= neckJointIsPitch[i];
         }
         hasPitchJoint = pitchJointFound;
         initializePitchLimits();
      }
      else
      {
         neckJointNamesArray = new NeckJointName[0];
         neckJointIsPitch = new boolean[0];
         hasPitchJoint = false;
      }
   }

   public void renderImGuiWidgets()
   {
      if (!hasPitchJoint)
         return;

      int currentPitchDeg = getCurrentPitchDeg();
      ImGui.beginDisabled(!syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0));
      boolean changed = ImGui.sliderInt(labels.get(sliderName), sliderValue.getData(), minPitchDeg, maxPitchDeg);
      if (!changed && !ImGui.isItemActive())
         sliderValue.set(currentPitchDeg);
      ImGui.endDisabled();
      if (changed && sendThrottler.run(sendPeriod))
      {
         NeckJointName[] neckJointNames = syncedRobot.getRobotModel().getJointMap().getNeckJointNames();
         double desiredPitchRad = Math.toRadians(sliderValue.get());
         double[] desiredNeckJointValues = new double[neckJointNames.length];
         for (int i = 0; i < neckJointNames.length; i++)
         {
            OneDoFJointBasics neckJoint = syncedRobot.getFullRobotModel().getNeckJoint(neckJointNames[i]);
            desiredNeckJointValues[i] = neckJoint != null ? neckJoint.getQ() : 0.0;
            if (neckJointNames[i] == NeckJointName.DISTAL_NECK_PITCH || neckJointNames[i] == NeckJointName.PROXIMAL_NECK_PITCH)
               desiredNeckJointValues[i] = desiredPitchRad;
         }

         ros2ControllerHelper.publishToController(HumanoidMessageTools.createHeadJointspaceTaskspaceTrajectoryMessage(syncedRobot.getReferenceFrames(),
                                                                                                                      neckJointNames,
                                                                                                                      desiredNeckJointValues,
                                                                                                                      trajectoryTimeSeconds));
      }
   }

   private int getCurrentPitchDeg()
   {
      double pitchSum = 0.0;
      int pitchCount = 0;
      for (int i = 0; i < neckJointNamesArray.length; i++)
      {
         if (!neckJointIsPitch[i])
            continue;
         OneDoFJointBasics neckJoint = syncedRobot.getFullRobotModel().getNeckJoint(neckJointNamesArray[i]);
         if (neckJoint == null)
            continue;
         pitchSum += neckJoint.getQ();
         pitchCount++;
      }
      if (pitchCount == 0)
         return 0;
      return (int) Math.round(Math.toDegrees(pitchSum / pitchCount));
   }

   private void initializePitchLimits()
   {
      for (int i = 0; i < neckJointNamesArray.length; i++)
      {
         if (!neckJointIsPitch[i])
            continue;
         OneDoFJointBasics neckJoint = syncedRobot.getFullRobotModel().getNeckJoint(neckJointNamesArray[i]);
         if (neckJoint != null)
         {
            minPitchDeg = (int) Math.round(Math.toDegrees(neckJoint.getJointLimitLower()));
            maxPitchDeg = (int) Math.round(Math.toDegrees(neckJoint.getJointLimitUpper()));
            return;
         }
      }
   }
}
