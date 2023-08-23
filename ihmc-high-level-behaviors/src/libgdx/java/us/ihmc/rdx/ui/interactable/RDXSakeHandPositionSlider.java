package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.RobotConfigurationData;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXSakeHandPositionSlider
{
   private static final double UPDATE_PERIOD = UnitConversions.hertzToSeconds(10.0);
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double ROBOT_DATA_EXPIRATION_DURATION = 1.0;
   private static final double ANGLE_AT_CLOSE = Math.toRadians(-3.0);
   private static final double MAX_ANGLE_LIMIT = Math.toRadians(210.0);
   private static final double MIN_ANGLE_LIMIT = Math.toRadians(0.0);
   private static final double EPSILON = 1E-6;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final CommunicationHelper communicationHelper;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private double valueFromRobot = Double.NaN;
   private final RobotSide handSide;
   private final Throttler updateThrottler = new Throttler();
   private final Throttler sendThrottler = new Throttler();

   public RDXSakeHandPositionSlider(ROS2SyncedRobotModel syncedRobot,
                                    CommunicationHelper communicationHelper,
                                    RobotSide handSide)
   {
      this.syncedRobot = syncedRobot;
      this.communicationHelper = communicationHelper;
      this.handSide = handSide;
      sliderName = handSide.getPascalCaseName() + " angle";

      syncedRobot.addRobotConfigurationDataReceivedCallback(this::receiveRobotConfigurationData);
   }

   private void receiveRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (updateThrottler.run(UPDATE_PERIOD) && syncedRobot.getLatestHandJointAnglePacket(handSide) != null)
      {
         valueFromRobot = syncedRobot.getLatestHandJointAnglePacket(handSide).getJointAngles().get(0);
      }
   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(SEND_PERIOD) && syncedRobot.getDataReceptionTimerSnapshot().isRunning(ROBOT_DATA_EXPIRATION_DURATION))
         {
            double positionRatio = sliderValue[0] / MAX_ANGLE_LIMIT;
            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic,
                                        HumanoidMessageTools.createHandSakeDesiredCommandMessage(handSide,
                                                                                                 (byte) 5, // TODO: Get access to SakeHandCommand.HandConfiguration
                                                                                                 positionRatio,
                                                                                                 0.3));
         }
      }
      else
      {
         sliderValue[0] = (float) (2.0 * (valueFromRobot - ANGLE_AT_CLOSE));
      }
   }

   private boolean renderImGuiSliderAndReturnChanged()
   {
      float previousValue = sliderValue[0];
      ImGui.sliderAngle(labels.get(sliderName), sliderValue, (float) Math.toDegrees(MIN_ANGLE_LIMIT), (float) Math.toDegrees(MAX_ANGLE_LIMIT));
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && !MathTools.epsilonEquals(currentValue, previousValue, EPSILON);
   }
}