package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.YawPitchRollAxis;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXChestOrientationSlider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final YawPitchRollAxis yawPitchRollAxis;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final String sliderName;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final float[] sliderValue = new float[1];
   private double robotDataExpirationDuration = 1.0;
   private double minAngle = -Math.toRadians(5.0);
   private double maxAngle = Math.toRadians(5.0);
   private volatile double valueFromRobot = Double.NaN;
   private final Throttler updateThrottler = new Throttler();
   private double updatePeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler sendThrottler = new Throttler();
   private double sendPeriod = UnitConversions.hertzToSeconds(5.0);

   public RDXChestOrientationSlider(ROS2SyncedRobotModel syncedRobot,
                                    YawPitchRollAxis yawPitchRollAxis,
                                    ROS2ControllerHelper ros2ControllerHelper,
                                    RDXTeleoperationParameters teleoperationParameters)
   {
      this.syncedRobot = syncedRobot;
      this.yawPitchRollAxis = yawPitchRollAxis;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;
      sliderName = "Chest " + yawPitchRollAxis.getPascalCasedName();

      syncedRobot.addRobotConfigurationDataReceivedCallback(this::receiveRobotConfigurationData);
   }

   private void receiveRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (updateThrottler.run(updatePeriod))
      {
         FrameYawPitchRoll chestFrame = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
         chestFrame.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
         valueFromRobot = yawPitchRollAxis.getFromYawPitchRoll(chestFrame);
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

               FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
               frameChestYawPitchRoll.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
               frameChestYawPitchRoll.set(yawPitchRollAxis.createYawPitchRoll(desiredAngle));
               frameChestYawPitchRoll.changeFrame(ReferenceFrame.getWorldFrame());

               LogTools.info("Commanding chest {}: {}", yawPitchRollAxis.getLowerCasedName(), desiredAngle);

               ChestTrajectoryMessage message = new ChestTrajectoryMessage();

               double change = Math.abs(desiredAngle - valueFromRobot);
               double trajectoryTime = change * teleoperationParameters.getChestOrientationVelocity() * sendPeriod;
               trajectoryTime = MathTools.clamp(trajectoryTime, 0.5, 1000.0); // Safety

               message.getSo3Trajectory()
                      .set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryTime,
                                                                           frameChestYawPitchRoll,
                                                                           EuclidCoreTools.zeroVector3D,
                                                                           syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
               message.getSo3Trajectory().getSelectionMatrix().setXSelected(false);
               message.getSo3Trajectory().getSelectionMatrix().setYSelected(false);
               message.getSo3Trajectory().getSelectionMatrix().setZSelected(false);
               switch (yawPitchRollAxis)
               {
                  case YAW -> message.getSo3Trajectory().getSelectionMatrix().setZSelected(true);
                  case PITCH -> message.getSo3Trajectory().getSelectionMatrix().setYSelected(true);
                  case ROLL -> message.getSo3Trajectory().getSelectionMatrix().setXSelected(true);
               }
               long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
               message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

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
//      if (yawPitchRollAxis == YawPitchRollAxis.PITCH)
//      {
//         minAngle = teleoperationParameters.getChestMinimumPitch();
//         maxAngle = teleoperationParameters.getChestMaximumPitch();
//      }
//      else if (yawPitchRollAxis == YawPitchRollAxis.YAW)
//      {
//         minAngle = teleoperationParameters.getChestMinimumYaw();
//         maxAngle = teleoperationParameters.getChestMaximumYaw();
//      }

      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, (float) minAngle, (float) maxAngle);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && currentValue != previousValue;
   }
}
