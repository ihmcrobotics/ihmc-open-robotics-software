package us.ihmc.rdx.ui.interactable;

import controller_msgs.PelvisOrientationTrajectoryMessage;
import controller_msgs.RobotConfigurationData;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.geometry.YawPitchRollAxis;

/**
 * Teleoperation slider that commands a single pelvis-orientation axis (pitch by default; roll/yaw supported for free)
 * as a {@link PelvisOrientationTrajectoryMessage}. The pelvis pitch is measured/commanded relative to the level,
 * heading-aligned mid-feet frame, so the operator's commanded value is the pelvis lean angle off vertical. On the MPC
 * side this drives the pelvis-rotation convergence cost directly (the sole base-orientation actuator); the CoM
 * convergence cost holds the CoM over the feet, so leaning forward produces a balanced hip-hinge.
 * <p>
 * {@code enableUserPelvisControlDuringWalking} is set so the pelvis-orientation manager holds USER mode -- the lean
 * persists when the operator subsequently streams an arm trajectory rather than being reset to level.
 * </p>
 */
public class RDXPelvisOrientationSlider
{
   // Conservative default lean range. Start small; widen once the headless feasibility check confirms a larger pitch
   // is stable. Could be promoted to RDXTeleoperationParameters later.
   private static final double MIN_PITCH = -0.10; // rad (slight backward)
   private static final double MAX_PITCH = 0.45;  // rad (forward lean to reach down)
   private static final double PITCH_RATE = 0.75; // rad/s, used to size the trajectory time per slider increment

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final YawPitchRollAxis yawPitchRollAxis;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private final double robotDataExpirationDuration = 1.0;
   private volatile double valueFromRobot = Double.NaN;
   private final Throttler updateThrottler = new Throttler();
   private final double updatePeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler sendThrottler = new Throttler();
   private final double sendPeriod = UnitConversions.hertzToSeconds(5.0);

   public RDXPelvisOrientationSlider(ROS2SyncedRobotModel syncedRobot,
                                     YawPitchRollAxis yawPitchRollAxis,
                                     ROS2ControllerHelper ros2ControllerHelper)
   {
      this.syncedRobot = syncedRobot;
      this.yawPitchRollAxis = yawPitchRollAxis;
      this.ros2ControllerHelper = ros2ControllerHelper;
      sliderName = "Pelvis " + yawPitchRollAxis.getPascalCasedName();

      syncedRobot.addRobotConfigurationDataReceivedCallback(this::receiveRobotConfigurationData);
   }

   private void receiveRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (updateThrottler.run(updatePeriod))
      {
         FrameYawPitchRoll pelvisFrame = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getPelvisFrame());
         pelvisFrame.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
         valueFromRobot = yawPitchRollAxis.getFromYawPitchRoll(pelvisFrame);
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
               double desiredAngle = sliderValue[0];

               // Command the chosen axis relative to the level, heading-aligned mid-feet frame, then express in world.
               FrameYawPitchRoll framePelvisYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
               framePelvisYawPitchRoll.set(yawPitchRollAxis.createYawPitchRoll(desiredAngle));
               framePelvisYawPitchRoll.changeFrame(ReferenceFrame.getWorldFrame());

               LogTools.info("Commanding pelvis {}: {}", yawPitchRollAxis.getLowerCasedName(), desiredAngle);

               double change = Math.abs(desiredAngle - valueFromRobot);
               double trajectoryTime = MathTools.clamp(change / PITCH_RATE, 0.3, 1000.0); // Safety

               PelvisOrientationTrajectoryMessage message =
                     HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime, framePelvisYawPitchRoll);
               // Keep the pelvis-orientation manager in USER mode so the lean is not reset when an arm trajectory streams.
               message.setEnableUserPelvisControlDuringWalking(true);

               ros2ControllerHelper.publishToController(message);
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
      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, (float) MIN_PITCH, (float) MAX_PITCH);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && currentValue != previousValue;
   }
}
