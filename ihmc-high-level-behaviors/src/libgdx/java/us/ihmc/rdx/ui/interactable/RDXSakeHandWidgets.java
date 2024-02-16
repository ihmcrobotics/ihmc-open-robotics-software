package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.EuclidCoreMissingTools;
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
   private boolean isCalibrated = false;
   private boolean needsReset;
   private double currentTemperature = Double.NaN;
   private double currentHandOpenAngle = Double.NaN;
   private double commandedHandOpenAngle = Double.NaN;
   private double currentFingertipGripForce = Double.NaN;
   private double commandedFingertipGripForceLimit = Double.NaN;
   private final RobotSide handSide;
   private final Throttler sendThrottler = new Throttler();
   private final Timer sentCommandFreezeExpiration = new Timer();
   private final SakeHandPreset[] presetButtons = new SakeHandPreset[] {SakeHandPreset.OPEN, SakeHandPreset.CLOSE, SakeHandPreset.GRIP};
   private final Notification userChangedHandOpenAngle = new Notification();
   private final Notification userChangedFingertipGripForce = new Notification();
   private final Notification calibrateRequested = new Notification();
   private final Notification resetErrorsRequested = new Notification();
   private final SakeHandDesiredCommandMessage sakeHandDesiredCommandMessage = new SakeHandDesiredCommandMessage();
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiFlashingText calibrateStatusText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText needResetStatusText = new ImGuiFlashingText(ImGuiTools.RED);

   public RDXSakeHandWidgets(CommunicationHelper communicationhelper, RobotSide handSide)
   {
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;

      handOpenAngleSliderLabel = "Hand Open Angle";
      handOpenAngleSlider = new ImGuiSliderDouble(handOpenAngleSliderLabel, "", Double.NaN);
      handOpenAngleSlider.addWidgetAligner(widgetAligner);
      fingertipGripForceSliderLabel = "Fingertip Grip Force Limit";
      fingertipGripForceSlider = new ImGuiSliderDouble(fingertipGripForceSliderLabel, "%.1f N", Double.NaN);
      fingertipGripForceSlider.addWidgetAligner(widgetAligner);

      communicationHelper.subscribeViaVolatileCallback(robotName -> ROS2Tools.getHandSakeStatusTopic(robotName, handSide), sakeHandStatusMessage ->
      {
         isCalibrated = sakeHandStatusMessage.getIsCalibrated();
         needsReset = sakeHandStatusMessage.getNeedsReset();
         currentTemperature = sakeHandStatusMessage.getTemperature();
         currentHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getNormalizedCurrentPosition());
         commandedHandOpenAngle = SakeHandParameters.denormalizeHandOpenAngle(sakeHandStatusMessage.getNormalizedDesiredPosition());
         currentFingertipGripForce = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getNormalizedCurrentTorque());
         commandedFingertipGripForceLimit = SakeHandParameters.denormalizeFingertipGripForceLimit(sakeHandStatusMessage.getNormalizedTorqueLimit());
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
         {
            LogTools.info("Commanding hand open angle %.1f%s".formatted(Math.toDegrees(handOpenAngleSlider.getDoubleValue()),
                                                                        EuclidCoreMissingTools.DEGREE_SYMBOL));
            sakeHandDesiredCommandMessage.setNormalizedGripperDesiredPosition(SakeHandParameters.normalizeHandOpenAngle(handOpenAngleSlider.getDoubleValue()));
         }

         if (sendForce)
         {
            LogTools.info("Commanding fingertip grip force limit %.1f N".formatted(fingertipGripForceSlider.getDoubleValue()));
            sakeHandDesiredCommandMessage.setNormalizedGripperTorqueLimit(
                  SakeHandParameters.normalizeFingertipGripForceLimit(fingertipGripForceSlider.getDoubleValue()));
         }

         if (sendCalibrate)
         {
            LogTools.info("Requesting hand calibration");
            sakeHandDesiredCommandMessage.setRequestCalibration(true);
         }

         if (sendResetErrors)
         {
            LogTools.info("Requesting hand to reset errors");
            sakeHandDesiredCommandMessage.setRequestResetErrors(true);
         }

         if (sendAngle || sendForce || sendCalibrate || sendResetErrors)
         {
            communicationHelper.publish(robotName -> ROS2Tools.getHandSakeCommandTopic(robotName, handSide), sakeHandDesiredCommandMessage);
         }
      }
   }

   public void renderImGuiWidgets()
   {
      for (SakeHandPreset preset : presetButtons)
      {
         if (ImGui.button(labels.get(preset.getPascalCasedName())))
         {
            handOpenAngleSlider.setDoubleValue(preset.getHandOpenAngle());
            fingertipGripForceSlider.setDoubleValue(preset.getFingertipGripForceLimit());
            userChangedHandOpenAngle.set();
            userChangedFingertipGripForce.set();
            sentCommandFreezeExpiration.reset();
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

      calibrateStatusText.renderText("Is Calibrated: %b ".formatted(isCalibrated), !isCalibrated);
      ImGui.sameLine();
      needResetStatusText.renderText("Needs Reset: %b ".formatted(needsReset), needsReset);

      double currentHandOpenAngleNotchNormal = Math.abs(SakeHandParameters.normalizeHandOpenAngle(currentHandOpenAngle));

      float sliderStart = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
      float sliderEnd = ImGui.getColumnWidth();
      float sliderWidth = sliderEnd - sliderStart;

      ImGuiTools.renderSliderOrProgressNotch(sliderStart + (float) currentHandOpenAngleNotchNormal * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

      handOpenAngleSlider.setWidgetText("%.1f%s".formatted(Math.toDegrees(handOpenAngleSlider.getDoubleValue()), EuclidCoreMissingTools.DEGREE_SYMBOL));

      if (handOpenAngleSlider.render(0.0, Math.toRadians(SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES)))
      {
         userChangedHandOpenAngle.set();
         sentCommandFreezeExpiration.reset();
      }

      double currentForceNotchNormal = Math.abs(SakeHandParameters.normalizeFingertipGripForceLimit(currentFingertipGripForce));
      double moderateForceNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD);
      double highForceNotchNormal = SakeHandParameters.normalizeFingertipGripForceLimit(SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD);

      sliderStart = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
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

      fingertipGripForceSlider.setWidgetText("%+.1f / %.1f N".formatted(currentFingertipGripForce, fingertipGripForceSlider.getDoubleValue()));

      if (fingertipGripForceSlider.render(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT))
      {
         userChangedFingertipGripForce.set();
         sentCommandFreezeExpiration.reset();
      }

      if (styled)
         ImGui.popStyleColor(2);

      if (currentTemperature >= SakeHandParameters.ERROR_TEMPERATURE_CELCIUS)
         ImGui.pushStyleColor(ImGuiCol.PlotHistogram, ImGuiTools.RED);
      else if (currentTemperature >= SakeHandParameters.WARNING_TEMPERATURE_CELCIUS)
         ImGui.pushStyleColor(ImGuiCol.PlotHistogram, ImGuiTools.YELLOW);
      else
         ImGui.pushStyleColor(ImGuiCol.PlotHistogram, ImGuiTools.LIGHT_GRAY);

      widgetAligner.text("Temperature");
      ImGui.progressBar((float) (currentTemperature / SakeHandParameters.DYNAMIXEL_FAILURE_TEMPERATURE_CELCIUS),
                        ImGui.getColumnWidth(),
                        ImGui.getFrameHeight(),
                        "%.1f %sC".formatted(currentTemperature, EuclidCoreMissingTools.DEGREE_SYMBOL));

      ImGui.popStyleColor();
   }

   public boolean getCalibrated()
   {
      return isCalibrated;
   }

   public boolean getNeedsReset()
   {
      return needsReset;
   }
}
