package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXSakeHandTorqueSlider
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double ROBOT_DATA_EXPIRATION_DURATION = 1.0;
   private static final double FREEZE_DURATION = 1.0;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final CommunicationHelper communicationHelper;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private double currentFingertipGripForce = Double.NaN;
   private double commandedFingertipGripForceStatus = Double.NaN;
   private final RobotSide handSide;
   private final Throttler sendThrottler = new Throttler();
   private final Timer sentCommandFreezeExpiration = new Timer();

   public RDXSakeHandTorqueSlider(ROS2SyncedRobotModel syncedRobot, CommunicationHelper communicationhelper, RobotSide handSide)
   {
      this.syncedRobot = syncedRobot;
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;

      sliderName = handSide.getPascalCaseName() + " torque";

      communicationHelper.subscribeViaCallback(ROS2Tools::getHandSakeStatusTopic, message ->
      {
         if (message.getRobotSide() == handSide.toByte())
         {
            currentFingertipGripForce = SakeHandParameters.denormalizeFingertipGripForceLimit(message.getPresentTorqueRatio());
            commandedFingertipGripForceStatus = SakeHandParameters.denormalizeFingertipGripForceLimit(message.getGoalTorqueRatio());
         }
      });
   }

   public void renderImGuiWidgets()
   {
      int notchColor = ImGuiTools.greenToRedGradiatedColor(currentFingertipGripForce, 0.5, 0.7, 0.9);
      float textSizeX = ImGuiTools.calcTextSizeX(sliderName);
      float currentValueNotchX = (float) ((ImGui.getItemRectSizeX() - textSizeX - 12.0f) * currentFingertipGripForce);

      ImGui.getWindowDrawList().addRectFilled(ImGui.getCursorScreenPosX() + 2.0f,
                                              ImGui.getCursorScreenPosY() + 2.0f,
                                              ImGui.getCursorScreenPosX() + currentValueNotchX + 12.0f,
                                              ImGui.getCursorScreenPosY() + 20.0f,
                                              notchColor);

      if (sentCommandFreezeExpiration.isExpired(FREEZE_DURATION))
      {
         sliderValue[0] = (float) commandedFingertipGripForceStatus;
      }

      ImGui.sliderFloat(labels.get(sliderName),
                        sliderValue,
                        0.0f,
                        (float) SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT,
                        String.format("%.1f N", (sliderValue[0])));

      if (!Double.isNaN(sliderValue[0]) && sliderValue[0] != (float) commandedFingertipGripForceStatus)
      {
         if (sendThrottler.run(SEND_PERIOD))
         {
            LogTools.info("Sending");
            SakeHandDesiredCommandMessage message = new SakeHandDesiredCommandMessage();
            message.setRobotSide(handSide.toByte());
            message.setDesiredHandConfiguration((byte) SakeHandCommandOption.GOTO.getCommandNumber());
            message.setPostionRatio(-1.0);
            message.setTorqueRatio(SakeHandParameters.normalizeFingertipGripForceLimit(sliderValue[0]));
            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);
            sentCommandFreezeExpiration.reset();
         }
      }
   }
}
