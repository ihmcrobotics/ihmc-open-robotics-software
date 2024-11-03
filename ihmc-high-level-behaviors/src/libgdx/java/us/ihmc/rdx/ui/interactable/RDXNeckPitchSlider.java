package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.RobotConfigurationData;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXNeckPitchSlider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final String sliderName;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final float[] sliderValue = new float[1];
   private double robotDataExpirationDuration = 1.0;
   private double minHeight = -Math.toRadians(5.0);
   private double maxHeight = Math.toRadians(5.0);
   private volatile double valueFromRobot = Double.NaN;
   private final Throttler updateThrottler = new Throttler();
   private double updatePeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler sendThrottler = new Throttler();
   private double sendPeriod = UnitConversions.hertzToSeconds(5.0);

   public RDXNeckPitchSlider(ROS2SyncedRobotModel syncedRobot,
                             ROS2ControllerHelper ros2ControllerHelper,
                             RDXTeleoperationParameters teleoperationParameters)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;
      sliderName = "Neck pitch";

      syncedRobot.addRobotConfigurationDataReceivedCallback(this::receiveRobotConfigurationData);
   }

   private void receiveRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (updateThrottler.run(updatePeriod))
      {
         //      neckJoint = fullRobotModel.getNeckJoint(NeckJointName.PROXIMAL_NECK_PITCH);
         //      if (neckJoint != null)
         //      {
         //         double neckJointLimitUpper = neckJoint.getJointLimitUpper();
         //         neckJointJointLimitLower = neckJoint.getJointLimitLower();
         //         neckJointRange = neckJointLimitUpper - neckJointJointLimitLower;
         //      }
      }
   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(sendPeriod))
         {
            if (syncedRobot.getDataReceptionTimerSnapshot().isRunning(robotDataExpirationDuration))
            {
               //      if (neckJoint != null && imGuiSlider("Neck Pitch", neckPitchSliderValue))
               //      {
               //         double percent = neckPitchSliderValue[0] / 100.0;
               //         percent = 1.0 - percent;
               //         MathTools.checkIntervalContains(percent, 0.0, 1.0);
               //         double jointAngle = neckJointJointLimitLower + percent * neckJointRange;
               //         LogTools.info("Commanding neck trajectory: slider: {} angle: {}", neckPitchSliderValue[0], jointAngle);
               //         communicationHelper.publishToController(HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[] {jointAngle}));
               //      }
            }
         }
      }
      else
      {
         sliderValue[0] = (float) valueFromRobot;
      }
   }

   private boolean renderImGuiSliderAndReturnChanged()
   {
      minHeight = teleoperationParameters.getPelvisMinimumHeight();
      maxHeight = teleoperationParameters.getPelvisMaximumHeight();
      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, (float) minHeight, (float) maxHeight);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && currentValue != previousValue;
   }
}
