package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXSakeHandTorqueSlider
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double FREEZE_DURATION = 1.0;

   private final CommunicationHelper communicationHelper;
   private final String sliderLabel;
   private final ImGuiSliderDouble slider;
   private double currentFingertipGripForce = Double.NaN;
   private double commandedFingertipGripForceStatus = Double.NaN;
   private final RobotSide handSide;
   private final Throttler sendThrottler = new Throttler();
   private final Timer sentCommandFreezeExpiration = new Timer();
   private final Notification needToSendMessage = new Notification();

   public RDXSakeHandTorqueSlider(CommunicationHelper communicationhelper, RobotSide handSide)
   {
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;

      sliderLabel = "Fingertip Grip Force Limit";
      slider = new ImGuiSliderDouble(sliderLabel, "%.1f N", Double.NaN);

      communicationHelper.subscribeViaCallback(ROS2Tools::getHandSakeStatusTopic, message ->
      {
         if (message.getRobotSide() == handSide.toByte())
         {
            currentFingertipGripForce = SakeHandParameters.denormalizeFingertipGripForceLimit(message.getPresentTorqueRatio());
            commandedFingertipGripForceStatus = SakeHandParameters.denormalizeFingertipGripForceLimit(message.getGoalTorqueRatio());
         }
      });
   }

   public void update()
   {
      if (!sentCommandFreezeExpiration.isRunning(FREEZE_DURATION))
      {
         slider.setDoubleValue(commandedFingertipGripForceStatus);
      }

      if (sendThrottler.run(SEND_PERIOD) && needToSendMessage.poll() && !Double.isNaN(slider.getDoubleValue()))
      {
         SakeHandDesiredCommandMessage message = new SakeHandDesiredCommandMessage();
         message.setRobotSide(handSide.toByte());
         message.setDesiredHandConfiguration((byte) SakeHandCommandOption.GOTO.getCommandNumber());
         message.setPostionRatio(-1.0);
         message.setTorqueRatio(SakeHandParameters.normalizeFingertipGripForceLimit(slider.getDoubleValue()));
         communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);
      }
   }

   public void renderImGuiWidgets()
   {
      double currentNotchNormal = Math.abs(SakeHandParameters.normalizeFingertipGripForceLimit(currentFingertipGripForce));
      double moderateNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD);
      double highNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD);

      float sliderStart = ImGuiTools.calcTextSizeX(sliderLabel) + ImGui.getStyle().getItemSpacingX();
      float sliderEnd = ImGui.getColumnWidth();
      float sliderWidth = sliderEnd - sliderStart;

      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) currentNotchNormal * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) moderateNotchNormal * sliderWidth, ImGuiTools.DARK_ORANGE);
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) highNotchNormal * sliderWidth, ImGuiTools.DARK_RED);

      boolean styled = false;
      if (slider.getDoubleValue() >= SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD)
      {
         ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.DARK_RED);
         ImGui.pushStyleColor(ImGuiCol.SliderGrabActive, ImGuiTools.DARK_RED);
         styled = true;
      }
      else if (slider.getDoubleValue() >= SakeHandParameters.FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD)
      {
         ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.DARK_ORANGE);
         ImGui.pushStyleColor(ImGuiCol.SliderGrabActive, ImGuiTools.DARK_ORANGE);
         styled = true;
      }

      slider.setWidgetText("%.1f / %.1f N".formatted(currentFingertipGripForce, slider.getDoubleValue()));

      if (slider.render(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT))
      {
         needToSendMessage.set();
         sentCommandFreezeExpiration.reset();
      }

      if (styled)
         ImGui.popStyleColor(2);
   }
}
