package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.Throttler;

public class RDXPelvisHeightSlider
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

   public RDXPelvisHeightSlider(ROS2SyncedRobotModel syncedRobot,
                                ROS2ControllerHelper ros2ControllerHelper,
                                RDXTeleoperationParameters teleoperationParameters)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;
      sliderName = "Pelvis height";

      syncedRobot.addRobotConfigurationDataReceivedCallback(this::receiveRobotConfigurationData);
   }

   private void receiveRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (updateThrottler.run(updatePeriod))
      {
         double pelvisZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
         double midFeetZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
         double midFeetToPelvis = pelvisZ - midFeetZ;
         valueFromRobot = midFeetToPelvis;
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
               double desiredHeight = sliderValue[0];
               double pelvisZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
               double midFeetZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
               double desiredHeightInWorld = desiredHeight + midFeetZ;

               double change = Math.abs(desiredHeight - valueFromRobot);
               double trajectoryTime = change * teleoperationParameters.getPelvisHeightChangeVelocity() * sendPeriod;
               trajectoryTime = MathTools.clamp(trajectoryTime, 0.5, 1000.0); // Safety

               LogTools.info(StringTools.format3D("Commanding height trajectory. traj. time: {} desired: {} (pelvis - midFeetZ): {} in world: {}",
                                                  trajectoryTime,
                                                  desiredHeight,
                                                  pelvisZ - midFeetZ,
                                                  desiredHeightInWorld));

               PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
               message.getEuclideanTrajectory()
                      .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime,
                                                                                 new Point3D(0.0, 0.0, desiredHeightInWorld),
                                                                                 ReferenceFrame.getWorldFrame()));
               long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
               message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
               message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
               message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
               message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
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
      
//      minHeight = teleoperationParameters.getPelvisMinimumHeight();
//      maxHeight = teleoperationParameters.getPelvisMaximumHeight();
      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, (float) minHeight, (float) maxHeight);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && currentValue != previousValue;
   }
}
