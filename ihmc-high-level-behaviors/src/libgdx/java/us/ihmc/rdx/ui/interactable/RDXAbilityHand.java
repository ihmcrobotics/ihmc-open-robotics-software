package us.ihmc.rdx.ui.interactable;

import ihmc_psyonic_ros2.msg.dds.AbilityHandState;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.psyonicros2.AbilityHandManager.ControlMode;
import us.ihmc.psyonicros2.AbilityHandManager.Grip;
import us.ihmc.psyonicros2.AbilityHandROS2HardwareCommunication;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.imgui.ImGuiTools;

import java.util.Arrays;

import static us.ihmc.psyonicros2.AbilityHandInterface.ACTUATOR_COUNT;

public class RDXAbilityHand
{
   private final String serialNumber;

   private static final float START_POSITION = 30.0f;
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 120.0f;
   private static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiSliderFloat controlFingersSlider;
   private final ImGuiSliderFloat[] fingerSliders = new ImGuiSliderFloat[ACTUATOR_COUNT];

   private final Notification commandNotification = new Notification();
   private final Throttler publishThrottler = new Throttler();


   private ControlMode controlMode;
   private AbilityHandState state;

   private final float[] currentPosition = new float[ACTUATOR_COUNT];
   private final float[] goalPosition = new float[ACTUATOR_COUNT];
   private final float[] goalVelocity = new float[ACTUATOR_COUNT];

   float[] actuatorPostions = new float[ACTUATOR_COUNT];

   public RDXAbilityHand(String serialNumber, AbilityHandROS2HardwareCommunication communication)
   {
      this.serialNumber = serialNumber;

      controlFingersSlider = new ImGuiSliderFloat("Control Fingers", "%.1f°", Float.NaN);
      controlFingersSlider.addWidgetAligner(widgetAligner);
      controlMode = ControlMode.POSITION;

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         currentPosition[i] = START_POSITION;
         goalPosition[i] = (i == 5) ? -START_POSITION : START_POSITION;
         goalVelocity[i] = 0;
         String label = FINGER_NAMES[i];
         fingerSliders[i] = new ImGuiSliderFloat(label, "%.1f°", Float.NaN);
         fingerSliders[i].addWidgetAligner(widgetAligner);
         fingerSliders[i].setFloatValue(currentPosition[i]);
         if (communication.getAvailableHandSerialNumbers().contains(serialNumber))
            communication.getCommand(serialNumber).getGoalPositions()[i] = goalPosition[i];
      }
      publishThrottler.setFrequency(30.0);
   }

   public void update(AbilityHandROS2HardwareCommunication communication)
   {
      state = communication.readState(serialNumber);
      actuatorPostions = state.getActuatorPositions();

      if(publishThrottler.run() && commandNotification.poll())
      {
         communication.publishCommand(serialNumber);
      }
   }

   public void renderImGuiWidgets(AbilityHandROS2HardwareCommunication communication)
   {
      if (!communication.getAvailableHandSerialNumbers().contains(serialNumber))
         return;
      communication.getCommand(serialNumber).setSerialNumber(serialNumber);
      if (ImGui.button("OPEN"))
      {
         gripMode(Grip.RELAX, communication);
      }
      ImGui.sameLine();
      if (ImGui.button("GRIP"))
      {
         gripMode(Grip.POWER, communication);
      }

      ImGui.sameLine();
      if (ImGui.button("TRIPOD CLOSED"))
      {
         gripMode(Grip.TRIPOD_C, communication);
      }
      ImGui.sameLine();
      if (ImGui.button("HOOK"))
      {
         gripMode(Grip.HOOK, communication);
      }
      if (ImGui.collapsingHeader("Other Grips"))
      {
         if (ImGui.button("TRIPOD OPEN"))
         {
            gripMode(Grip.TRIPOD_O, communication);
         }
         ImGui.sameLine();
         if (ImGui.button("PINCH OPEN"))
         {
            gripMode(Grip.PINCH_O, communication);
         }
         ImGui.sameLine();
         if (ImGui.button("PINCH CLOSED"))
         {
            gripMode(Grip.TRIPOD_C, communication);
         }
         if (ImGui.button("KEY"))
         {
            gripMode(Grip.KEY, communication);
         }
         ImGui.sameLine();
         if (ImGui.button("RUDE"))
         {
            gripMode(Grip.RUDE, communication);
         }
      }

      float currentNotch = (actuatorPostions[0] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
      float sliderStart = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
      float sliderWidth = ImGui.getColumnWidth() - sliderStart;
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

      if (controlFingersSlider.render(SLIDER_MIN, SLIDER_MAX))
      {
         float newPos = controlFingersSlider.getFloatValue();
         controlMode = ControlMode.POSITION;
         communication.getCommand(serialNumber).setControlMode(controlMode.toByte());
         for (int i = 0; i < ACTUATOR_COUNT - 2; i++)
         {
            communication.getCommand(serialNumber).getGoalPositions()[i] = newPos;
         }
         commandNotification.set();
      }

      if (ImGui.collapsingHeader("Individual Finger Control"))
      {
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            float notchNorm = (i != 5) ?
                  (actuatorPostions[i] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN) :
                  (-actuatorPostions[i] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
            float startX = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
            float width = ImGui.getColumnWidth() - startX;
            ImGuiTools.renderSliderOrProgressNotch(startX + notchNorm * width, ImGui.getColorU32(ImGuiCol.Text));
            if (fingerSliders[i].render(SLIDER_MIN, SLIDER_MAX))
            {
               float val = fingerSliders[i].getFloatValue();
               float f = (i == 5) ? -val : val;
               controlMode = ControlMode.POSITION;
               communication.getCommand(serialNumber).setControlMode(controlMode.toByte());
               communication.getCommand(serialNumber).getGoalPositions()[i] = f;
               commandNotification.set();
            }
         }
      }
   }

   private void gripMode(Grip grip, AbilityHandROS2HardwareCommunication communication)
   {
      controlMode = ControlMode.GRIP;
      Arrays.fill(communication.getCommand(serialNumber).getGoalVelocities(), 30.0f);
      communication.getCommand(serialNumber).setControlMode(controlMode.toByte());
      communication.getCommand(serialNumber).setGrip(grip.toByte());

      commandNotification.set();
   }
}
