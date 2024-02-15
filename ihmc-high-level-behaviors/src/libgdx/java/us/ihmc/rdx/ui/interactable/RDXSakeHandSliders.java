package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.sakeGripper.SakeHandConfiguration;
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

public class RDXSakeHandSliders
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double FREEZE_DURATION = 1.0;

   private final CommunicationHelper communicationHelper;
   private final String fingertipGripForceSliderLabel;
   private final ImGuiSliderDouble fingertipGripForceSlider;
   private double currentFingertipGripForce = Double.NaN;
   private double commandedFingertipGripForceStatus = Double.NaN;
   private final RobotSide handSide;
   private final Throttler sendThrottler = new Throttler();
   private final Timer sentCommandFreezeExpiration = new Timer();
   private final Notification needToSendTorqueMessage = new Notification();
   private final SakeHandDesiredCommandMessage sakeHandDesiredCommandMessage = new SakeHandDesiredCommandMessage();

   public RDXSakeHandSliders(CommunicationHelper communicationhelper, RobotSide handSide)
   {
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;

      fingertipGripForceSliderLabel = "Fingertip Grip Force Limit";
      fingertipGripForceSlider = new ImGuiSliderDouble(fingertipGripForceSliderLabel, "%.1f N", Double.NaN);

      communicationHelper.subscribeViaVolatileCallback(ROS2Tools::getHandSakeStatusTopic, message ->
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
         fingertipGripForceSlider.setDoubleValue(commandedFingertipGripForceStatus);
      }

      if (sendThrottler.run(SEND_PERIOD))
      {
         boolean sendForceMessage = needToSendTorqueMessage.poll() && !Double.isNaN(fingertipGripForceSlider.getDoubleValue());
         if (sendForceMessage)
         {
            sakeHandDesiredCommandMessage.setRobotSide(handSide.toByte());
            sakeHandDesiredCommandMessage.setDesiredHandConfiguration(SakeHandConfiguration.GOTO_POSITION_WITH_TORQUE.toByte());
            sakeHandDesiredCommandMessage.setPostionRatio(-1.0);
            sakeHandDesiredCommandMessage.setTorqueRatio(SakeHandParameters.normalizeFingertipGripForceLimit(fingertipGripForceSlider.getDoubleValue()));
            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic, sakeHandDesiredCommandMessage);
         }
      }
   }

   public void renderImGuiWidgets()
   {
      double currentNotchNormal = Math.abs(SakeHandParameters.normalizeFingertipGripForceLimit(currentFingertipGripForce));
      double moderateNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD);
      double highNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD);

      float sliderStart = ImGuiTools.calcTextSizeX(fingertipGripForceSliderLabel) + ImGui.getStyle().getItemSpacingX();
      float sliderEnd = ImGui.getColumnWidth();
      float sliderWidth = sliderEnd - sliderStart;

      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) currentNotchNormal * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) moderateNotchNormal * sliderWidth, ImGuiTools.DARK_ORANGE);
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) highNotchNormal * sliderWidth, ImGuiTools.DARK_RED);

      boolean styled = false;
      if (fingertipGripForceSlider.getDoubleValue() >= SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD)
      {
         ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.DARK_RED);
         ImGui.pushStyleColor(ImGuiCol.SliderGrabActive, ImGuiTools.DARK_RED);
         styled = true;
      }
      else if (fingertipGripForceSlider.getDoubleValue() >= SakeHandParameters.FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD)
      {
         ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.DARK_ORANGE);
         ImGui.pushStyleColor(ImGuiCol.SliderGrabActive, ImGuiTools.DARK_ORANGE);
         styled = true;
      }

      fingertipGripForceSlider.setWidgetText("%.1f / %.1f N".formatted(currentFingertipGripForce, fingertipGripForceSlider.getDoubleValue()));

      if (fingertipGripForceSlider.render(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT))
      {
         needToSendTorqueMessage.set();
         sentCommandFreezeExpiration.reset();
      }

      if (styled)
         ImGui.popStyleColor(2);
   }
}
