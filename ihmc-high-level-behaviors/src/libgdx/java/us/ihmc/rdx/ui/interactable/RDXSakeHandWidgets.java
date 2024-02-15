package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.avatar.sakeGripper.SakeHandPresets;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXSakeHandWidgets
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double FREEZE_DURATION = 1.0;

   private final CommunicationHelper communicationHelper;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String handOpenAngleSliderLabel;
   private final ImGuiSliderDouble handOpenAngleSlider;
   private final String fingertipGripForceSliderLabel;
   private final ImGuiSliderDouble fingertipGripForceSlider;
   private double currentTemperature = Double.NaN;
   private double currentHandOpenAngle = Double.NaN;
   private double commandedHandOpenAngle = Double.NaN;
   private double currentFingertipGripForceLimit = Double.NaN;
   private double commandedFingertipGripForceLimit = Double.NaN;
   private final RobotSide handSide;
   private final Throttler sendThrottler = new Throttler();
   private final Timer sentCommandFreezeExpiration = new Timer();
   private final Notification userChangedHandOpenAngle = new Notification();
   private final Notification userChangedFingertipGripForce = new Notification();
   private final Notification calibrateRequested = new Notification();
   private final Notification resetErrorsRequested = new Notification();
   private final SakeHandDesiredCommandMessage sakeHandDesiredCommandMessage = new SakeHandDesiredCommandMessage();

   public RDXSakeHandWidgets(CommunicationHelper communicationhelper, RobotSide handSide)
   {
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;

      handOpenAngleSliderLabel = "Hand Open Angle";
      handOpenAngleSlider = new ImGuiSliderDouble(handOpenAngleSliderLabel, "", Double.NaN);
      fingertipGripForceSliderLabel = "Fingertip Grip Force Limit";
      fingertipGripForceSlider = new ImGuiSliderDouble(fingertipGripForceSliderLabel, "%.1f N", Double.NaN);

      communicationHelper.subscribeViaVolatileCallback(ROS2Tools::getHandSakeStatusTopic, sakeHandStatusMessage ->
      {
         if (sakeHandStatusMessage.getRobotSide() == handSide.toByte())
         {
            currentTemperature = sakeHandStatusMessage.getTemperature();
            currentHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getNormalizedCurrentPosition());
            commandedHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getNormalizedDesiredPosition());
            currentFingertipGripForceLimit = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getNormalizedCurrentPosition());
            commandedFingertipGripForceLimit = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getNormalizedTorqueLimit());
         }
      });

      sakeHandDesiredCommandMessage.setRobotSide(handSide.toByte());
   }

   public void update()
   {
      if (!sentCommandFreezeExpiration.isRunning(FREEZE_DURATION))
      {
         handOpenAngleSlider.setDoubleValue(commandedHandOpenAngle);
         fingertipGripForceSlider.setDoubleValue(commandedFingertipGripForceLimit);
      }

      if (sendThrottler.run(SEND_PERIOD))
      {
         boolean sendAngle = userChangedHandOpenAngle.poll() && !Double.isNaN(handOpenAngleSlider.getDoubleValue());
         boolean sendForce = userChangedFingertipGripForce.poll() && !Double.isNaN(fingertipGripForceSlider.getDoubleValue());
         boolean sendCalibrate = calibrateRequested.poll();
         boolean sendResetErrors = resetErrorsRequested.poll();

         sakeHandDesiredCommandMessage.setRobotSide(handSide.toByte());
         SakeHandParameters.resetDesiredCommandMessage(sakeHandDesiredCommandMessage);

         if (sendAngle)
            sakeHandDesiredCommandMessage.setNormalizedGripperDesiredPosition(SakeHandParameters.normalizeHandOpenAngle(handOpenAngleSlider.getDoubleValue()));

         if (sendForce)
            sakeHandDesiredCommandMessage.setNormalizedGripperTorqueLimit(
                  SakeHandParameters.normalizeFingertipGripForceLimit(fingertipGripForceSlider.getDoubleValue()));

         if (sendCalibrate)
            sakeHandDesiredCommandMessage.setRequestCalibration(true);

         if (sendResetErrors)
            sakeHandDesiredCommandMessage.setRequestResetErrors(true);

         if (sendAngle || sendForce || sendCalibrate || sendResetErrors)
         {
            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic, sakeHandDesiredCommandMessage);
            sentCommandFreezeExpiration.reset();
         }
      }
   }

   public void renderImGuiWidgets()
   {
      for (SakeHandPresets preset : SakeHandPresets.values)
      {
         if (ImGui.button(labels.get(preset.getPascalCasedName())))
         {
            handOpenAngleSlider.setDoubleValue(preset.getHandOpenAngle());
            fingertipGripForceSlider.setDoubleValue(preset.getFingertipGripForceLimit());
            userChangedHandOpenAngle.set();
            userChangedFingertipGripForce.set();
         }
         ImGui.sameLine();
      }

      if (ImGui.button(labels.get("Calibrate")))
      {
         calibrateRequested.set();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Reset Errors")))
      {
         resetErrorsRequested.set();
      }

      double currentHandOpenAngleNotchNormal = Math.abs(SakeHandParameters.normalizeHandOpenAngle(currentHandOpenAngle));

      float sliderStart = ImGuiTools.calcTextSizeX(handOpenAngleSliderLabel) + ImGui.getStyle().getItemSpacingX();
      float sliderEnd = ImGui.getColumnWidth();
      float sliderWidth = sliderEnd - sliderStart;

      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) currentHandOpenAngleNotchNormal * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

      if (handOpenAngleSlider.render(0.0, SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES))
      {
         userChangedHandOpenAngle.set();
      }

      double currentForceNotchNormal = Math.abs(SakeHandParameters.normalizeFingertipGripForceLimit(currentFingertipGripForceLimit));
      double moderateForceNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD);
      double highForceNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD);

      sliderStart = ImGuiTools.calcTextSizeX(fingertipGripForceSliderLabel) + ImGui.getStyle().getItemSpacingX();
      sliderEnd = ImGui.getColumnWidth();
      sliderWidth = sliderEnd - sliderStart;

      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) currentForceNotchNormal * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) moderateForceNotchNormal * sliderWidth, ImGuiTools.DARK_ORANGE);
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) highForceNotchNormal * sliderWidth, ImGuiTools.DARK_RED);

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

      fingertipGripForceSlider.setWidgetText("%.1f / %.1f N".formatted(currentFingertipGripForceLimit, fingertipGripForceSlider.getDoubleValue()));

      if (fingertipGripForceSlider.render(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT))
      {
         userChangedFingertipGripForce.set();
      }

      if (styled)
         ImGui.popStyleColor(2);
   }
}
