package us.ihmc.rdx.ui.hands.sakeEZGripper;

import ihmc_hands_ros2.EZGripperCommand;
import ihmc_hands_ros2.EZGripperState;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.handsros2.ezGripper.EZGripper.OperationMode;
import us.ihmc.handsros2.ezGripper.EZGripperError;
import us.ihmc.handsros2.ezGripper.EZGripperROS2HardwareCommunication;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.hands.RDXHandInterface;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXEZGripper implements RDXHandInterface
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final long CONNECTION_TIMEOUT = 500; // Hand connection timeout in millis

   private final RobotSide handSide;
   private final EZGripperROS2HardwareCommunication communication;

   private final EZGripperCommand commandMessage = new EZGripperCommand();
   private final Throttler sendThrottler = new Throttler();
   private final Notification commandChanged = new Notification();

   private final SakeHandPreset[] presetButtons = new SakeHandPreset[] {SakeHandPreset.OPEN, SakeHandPreset.CLOSE, SakeHandPreset.GRIP};

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSliderDouble handOpenAngleDegreesSlider;
   private final ImGuiSliderDouble fingertipGripForceSlider;

   private final ImBoolean imAutoCoolDownEnabled = new ImBoolean(true);
   private final ImInt temperatureLimit = new ImInt(75);

   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiFlashingText calibrateStatusText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText needResetStatusText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText sakeErrorStatusText = new ImGuiFlashingText(ImGuiTools.RED);

   private OperationMode previousOperationMode = null;

   public RDXEZGripper(RobotSide handSide, EZGripperROS2HardwareCommunication communication)
   {
      this.handSide = handSide;
      this.communication = communication;

      handOpenAngleDegreesSlider = new ImGuiSliderDouble("Hand Open Angle", "", Double.NaN);
      handOpenAngleDegreesSlider.addWidgetAligner(widgetAligner);
      fingertipGripForceSlider = new ImGuiSliderDouble("Fingertip Grip Force Limit", "%.1f N", Double.NaN);
      fingertipGripForceSlider.addWidgetAligner(widgetAligner);
   }

   @Override
   public RobotSide getSide()
   {
      return handSide;
   }

   @Override
   public boolean isCalibrated()
   {
      return communication.isHandConnected(handSide, CONNECTION_TIMEOUT) && communication.readState(handSide).getIsCalibrated();
   }

   @Override
   public boolean needsReset()
   {
      return communication.isHandConnected(handSide, CONNECTION_TIMEOUT) && communication.readState(handSide).getErrorCode() != EZGripperError.NONE.errorCode;
   }

   @Override
   public void update()
   {
      if (!communication.isHandConnected(handSide, CONNECTION_TIMEOUT))
         return;

      EZGripperState state = communication.readState(handSide);

      if (previousOperationMode != OperationMode.POSITION_CONTROL)
         handOpenAngleDegreesSlider.setDoubleValue(Math.toDegrees(SakeHandParameters.denormalizeHandOpenAngle(state.getCurrentPosition())));

      if (sendThrottler.run(SEND_PERIOD) && commandChanged.poll())
      {
         communication.publishCommand(handSide, commandMessage);
         previousOperationMode = OperationMode.fromByte(commandMessage.getOperationMode());

         // This prevents requesting calibration or error reset multiple times in a row
         if (previousOperationMode != OperationMode.POSITION_CONTROL)
         {
            commandMessage.setOperationMode(EZGripperCommand.POSITION_CONTROL);
            commandChanged.set();
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      EZGripperState state = communication.readState(handSide);

      ImGui.beginDisabled(needsReset() || !isCalibrated());

      for (SakeHandPreset preset : presetButtons)
      {
         if (ImGui.button(labels.get(preset.getPascalCasedName())))
         {
            double normalizedPosition = SakeHandParameters.normalizeHandOpenAngle(preset.getHandOpenAngle());
            double normalizedEffort = SakeHandParameters.normalizeFingertipGripForceLimit(preset.getFingertipGripForceLimit());

            // Set the command
            commandMessage.setOperationMode(EZGripperCommand.POSITION_CONTROL);
            commandMessage.setGoalPosition((float) normalizedPosition);
            commandMessage.setMaxEffort((float) normalizedEffort);
            commandMessage.setTorqueOn(normalizedEffort > 1E-3);

            // Update sliders to match
            handOpenAngleDegreesSlider.setDoubleValue(Math.toDegrees(preset.getHandOpenAngle()));
            fingertipGripForceSlider.setDoubleValue(preset.getFingertipGripForceLimit());

            commandChanged.set();
         }
         ImGui.sameLine();
      }

      ImGui.endDisabled();
      ImGui.beginDisabled(needsReset());

      if (ImGui.button(labels.get("Calibrate")))
      {
         commandMessage.setOperationMode(EZGripperCommand.CALIBRATION);
         commandChanged.set();
      }

      ImGui.endDisabled();

      ImGui.sameLine();
      if (ImGui.button(labels.get("Reset Errors")))
      {
         commandMessage.setOperationMode(EZGripperCommand.ERROR_RESET);
         commandChanged.set();
      }

      if (ImGui.checkbox(labels.get("Automatic cooldown enabled"), imAutoCoolDownEnabled))
         commandChanged.set();

      ImGui.sameLine();

      ImGui.beginDisabled(!imAutoCoolDownEnabled.get());
      if (ImGuiTools.volatileInputInt(labels.get("Temperature Limit"), temperatureLimit))
         commandChanged.set();
      ImGui.endDisabled();

      commandMessage.setTemperatureLimit((byte) (imAutoCoolDownEnabled.get() ? temperatureLimit.get() : 255));

      calibrateStatusText.renderText("Is Calibrated: %b ".formatted(isCalibrated()), !isCalibrated());
      ImGui.sameLine();
      needResetStatusText.renderText("Needs Reset: %b ".formatted(needsReset()), needsReset());

      String errorString = getErrorString(state.getErrorCode());
      if (!errorString.isEmpty())
      {
         ImGui.sameLine();
         sakeErrorStatusText.renderText("Error: %s".formatted(errorString), true);
      }

      if (state.getOperationMode() == EZGripperState.CALIBRATION)
      {
         ImGui.sameLine();
         ImGui.text("Calibrating...");
      }

      if (state.getOperationMode() == EZGripperState.COOLDOWN)
      {
         ImGui.sameLine();
         ImGui.text("Cooling down...");
      }

      ImGui.beginDisabled(needsReset() || !isCalibrated());

      float currentHandOpenAngleNotchNormal = Math.abs(state.getCurrentPosition());

      float sliderStart = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
      float sliderEnd = ImGui.getColumnWidth();
      float sliderWidth = sliderEnd - sliderStart;

      ImGuiTools.renderSliderOrProgressNotch(sliderStart + currentHandOpenAngleNotchNormal * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

      handOpenAngleDegreesSlider.setWidgetText("%.1f%s".formatted(handOpenAngleDegreesSlider.getDoubleValue(), EuclidCoreMissingTools.DEGREE_SYMBOL));

      if (handOpenAngleDegreesSlider.render(0.0, SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES))
      {
         double normalizedPosition = SakeHandParameters.normalizeHandOpenAngle(Math.toRadians(handOpenAngleDegreesSlider.getDoubleValue()));
         double normalizedEffort = SakeHandParameters.normalizeFingertipGripForceLimit(fingertipGripForceSlider.getDoubleValue());

         commandMessage.setOperationMode(EZGripperCommand.POSITION_CONTROL);
         commandMessage.setGoalPosition((float) normalizedPosition);
         commandMessage.setMaxEffort((float) normalizedEffort);
         commandMessage.setTorqueOn(normalizedEffort > 1E-3);

         commandChanged.set();
      }

      double currentForceNotchNormal = Math.abs(state.getCurrentEffort());
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

      fingertipGripForceSlider.setWidgetText("%+.1f / %.1f N".formatted(SakeHandParameters.denormalizeFingertipGripForceLimit(state.getCurrentEffort()),
                                                                        fingertipGripForceSlider.getDoubleValue()));

      if (fingertipGripForceSlider.render(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT))
      {
         double normalizedPosition = SakeHandParameters.normalizeHandOpenAngle(Math.toRadians(handOpenAngleDegreesSlider.getDoubleValue()));
         double normalizedEffort = SakeHandParameters.normalizeFingertipGripForceLimit(fingertipGripForceSlider.getDoubleValue());

         commandMessage.setOperationMode(EZGripperCommand.POSITION_CONTROL);
         commandMessage.setGoalPosition((float) normalizedPosition);
         commandMessage.setMaxEffort((float) normalizedEffort);
         commandMessage.setTorqueOn(normalizedEffort > 1E-3);

         commandChanged.set();
      }

      if (styled)
         ImGui.popStyleColor(2);

      ImGui.endDisabled();

      if (state.getTemperature() >= SakeHandParameters.ERROR_TEMPERATURE_CELSIUS)
         ImGui.pushStyleColor(ImGuiCol.PlotHistogram, ImGuiTools.RED);
      else if (state.getTemperature() >= SakeHandParameters.WARNING_TEMPERATURE_CELSIUS)
         ImGui.pushStyleColor(ImGuiCol.PlotHistogram, ImGuiTools.YELLOW);
      else
         ImGui.pushStyleColor(ImGuiCol.PlotHistogram, ImGuiTools.LIGHT_GRAY);

      widgetAligner.text("Temperature");
      ImGui.progressBar((float) (state.getTemperature() / SakeHandParameters.DYNAMIXEL_FAILURE_TEMPERATURE_CELSIUS),
                        ImGui.getColumnWidth(),
                        ImGui.getFrameHeight(),
                        "%d %sC".formatted(state.getTemperature(), EuclidCoreMissingTools.DEGREE_SYMBOL));

      ImGui.popStyleColor();
   }

   private String getErrorString(byte errorCodes)
   {
      StringBuilder errorString = new StringBuilder();
      for (String errorName : EZGripperError.getErrorNames(errorCodes))
         errorString.append("[").append(errorName).append("]");

      return errorString.toString();
   }

   @Override
   public void sendCommand(HandAction action)
   {
      if (!communication.isHandConnected(handSide, CONNECTION_TIMEOUT))
         return;

      switch (action)
      {
         case OPEN ->
         {
            EZGripperCommand command = commandMessage;
            command.setOperationMode(EZGripperCommand.POSITION_CONTROL);
            command.setGoalPosition((float) SakeHandParameters.normalizeHandOpenAngle(SakeHandPreset.OPEN.getHandOpenAngle()));
            command.setMaxEffort((float) (SakeHandPreset.OPEN.getFingertipGripForceLimit() / SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT));
            command.setTorqueOn(true);
            publishCommand();
         }
         case CLOSE ->
         {
            EZGripperCommand command = commandMessage;
            command.setOperationMode(EZGripperCommand.POSITION_CONTROL);
            command.setGoalPosition((float) SakeHandParameters.normalizeHandOpenAngle(SakeHandPreset.CLOSE.getHandOpenAngle()));
            command.setMaxEffort((float) (SakeHandPreset.CLOSE.getFingertipGripForceLimit() / SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT));
            command.setTorqueOn(true);
            publishCommand();
         }
         case GRIP ->
         {
            EZGripperCommand command = commandMessage;
            command.setOperationMode(EZGripperCommand.POSITION_CONTROL);
            command.setGoalPosition((float) SakeHandParameters.normalizeHandOpenAngle(SakeHandPreset.GRIP.getHandOpenAngle()));
            command.setMaxEffort((float) (SakeHandPreset.GRIP.getFingertipGripForceLimit() / SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT));
            command.setTorqueOn(true);
            publishCommand();
         }
         case CALIBRATE ->
         {
            EZGripperCommand command = commandMessage;
            command.setOperationMode(EZGripperCommand.CALIBRATION);
            publishCommand();
         }
         case RESET ->
         {
            EZGripperCommand command = commandMessage;
            command.setOperationMode(EZGripperCommand.ERROR_RESET);
            publishCommand();
         }
      }
   }

   private synchronized void publishCommand()
   {
      if (!communication.isHandConnected(handSide, CONNECTION_TIMEOUT))
         return;

      communication.publishCommand(handSide, commandMessage);
   }

   @Override
   public void sendFingerPosition(int index, float value)
   {
      //TODO implement
   }

   @Override
   public float getFingerPosition(int index)
   {
      //TODO implement
      return 0.0f;
   }
}
