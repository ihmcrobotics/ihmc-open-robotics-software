package us.ihmc.rdx.ui.hands.psyonicAbilityHand;

import ihmc_psyonic_ros2.msg.dds.AbilityHandCommand;
import ihmc_psyonic_ros2.msg.dds.AbilityHandState;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.log.LogTools;
import us.ihmc.psyonicros2.AbilityHandManager.ControlMode;
import us.ihmc.psyonicros2.AbilityHandManager.Grip;
import us.ihmc.psyonicros2.AbilityHandROS2HardwareCommunication;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.hands.RDXHandInterface;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Arrays;

import static us.ihmc.psyonicros2.AbilityHandInterface.ACTUATOR_COUNT;

public class RDXAbilityHand implements RDXHandInterface
{
   private static final float START_POSITION = 30.0f;
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 120.0f;
   private static final float GRIP_VELOCITY = 30.0f;
   private static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final String serialNumber;
   private final AbilityHandROS2HardwareCommunication communication;
   private final RobotSide handSide;

   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiSliderFloat controlFingersSlider;
   private final ImGuiSliderFloat[] fingerSliders = new ImGuiSliderFloat[ACTUATOR_COUNT];

   private final Notification commandNotification = new Notification();
   private final Throttler publishThrottler = new Throttler();

   private ControlMode controlMode;

   private float[] actuatorPostions = new float[ACTUATOR_COUNT];

   private ControlMode previousControl = null;

   public RDXAbilityHand(String serialNumber, AbilityHandROS2HardwareCommunication communication)
   {
      this.serialNumber = serialNumber;
      this.communication = communication;

      controlFingersSlider = new ImGuiSliderFloat("Control Fingers", "%.1f°", Float.NaN);
      controlFingersSlider.addWidgetAligner(widgetAligner);
      controlMode = ControlMode.POSITION;

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         String label = FINGER_NAMES[i];
         fingerSliders[i] = new ImGuiSliderFloat(label, "%.1f°", Float.NaN);
         fingerSliders[i].addWidgetAligner(widgetAligner);
         fingerSliders[i].setFloatValue(START_POSITION);
         communication.getCommand(serialNumber).getGoalPositions()[i] = (i == 5) ? -START_POSITION : START_POSITION;
      }

      AbilityHandState latestState = communication.readState(serialNumber);
      handSide = RobotSide.fromByte(latestState.getHandSide());

      publishThrottler.setFrequency(30.0);
   }

   @Override
   public void update()
   {
      actuatorPostions = communication.readState(serialNumber).getActuatorPositions();

      if(publishThrottler.run() && commandNotification.poll())
      {
         publishCommand();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.pushID(serialNumber);

      if (!communication.getAvailableHandSerialNumbers().contains(serialNumber))
         return;

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
            gripMode(Grip.PINCH_C, communication);
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
         if(previousControl != ControlMode.POSITION)
         {
            syncPositionSliders();
            previousControl = ControlMode.POSITION;
         }
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
               if(previousControl != ControlMode.POSITION)
               {
                  syncPositionSliders();
                  previousControl = ControlMode.POSITION;
               }
               float val = fingerSliders[i].getFloatValue();
               float f = (i == 5) ? -val : val;
               controlMode = ControlMode.POSITION;
               communication.getCommand(serialNumber).setControlMode(controlMode.toByte());
               communication.getCommand(serialNumber).getGoalPositions()[i] = f;
               commandNotification.set();
            }
         }
      }

      ImGui.popID();
   }

   private void gripMode(Grip grip, AbilityHandROS2HardwareCommunication communication)
   {
      Arrays.fill(communication.getCommand(serialNumber).getGoalVelocities(), GRIP_VELOCITY);
      communication.getCommand(serialNumber).setControlMode(ControlMode.GRIP.toByte());
      communication.getCommand(serialNumber).setGrip(grip.toByte());

      commandNotification.set();
      previousControl = ControlMode.GRIP;
   }

   private synchronized void publishCommand()
   {
      communication.publishCommand(serialNumber);
   }

   private void syncPositionSliders()
   {
      float[] cmdPos = communication.getCommand(serialNumber).getGoalPositions();
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         float live = actuatorPostions[i];
         cmdPos[i] = live;
         float val = (i == 5) ? -live : live;
         fingerSliders[i].setFloatValue(val);
      }
      controlFingersSlider.setFloatValue(cmdPos[0]);
   }

   @Override
   public RobotSide getSide()
   {
      return handSide;
   }

   @Override
   public boolean isCalibrated()
   {
      return true;
   }

   @Override
   public boolean needsReset()
   {
      return false;
   }

   @Override
   public void sendCommand(HandAction handAction)
   {
      byte abilityHandGrip;

      switch (handAction)
      {
         case OPEN -> abilityHandGrip = AbilityHandCommand.RELAX_GRIP;
         case CLOSE, GRIP -> abilityHandGrip = AbilityHandCommand.POWER_GRIP;
         default ->
         {
            LogTools.warn("Attempted to send an unsupported hand action command: {}", handAction.name());
            return;
         }
      }

      Arrays.fill(communication.getCommand(serialNumber).getGoalVelocities(), GRIP_VELOCITY);
      communication.getCommand(serialNumber).setControlMode(AbilityHandCommand.GRIP_CONTROL);
      communication.getCommand(serialNumber).setGrip(abilityHandGrip);
      previousControl = ControlMode.GRIP;
      publishCommand();
   }

   @Override
   public void sendFingerPosition(int index, float value)
   {
      if(previousControl != ControlMode.POSITION)
      {
         previousControl = ControlMode.POSITION;
      }

      float mappedValue;
      if (value < 0.05f)
      {
         mappedValue = 100.0f;
      }
      else if (value <= 0.85f)
      {
         // Linear scale from 100 -> 0 as value goes 0.05 -> 0.85
         // y = m * x + b
         float m = (0.0f - 100.0f) / (0.85f - 0.05f); // slope
         float b = 100.0f - m * 0.05f; // intercept
         mappedValue = m * value + b;
      }
      else
      {
         mappedValue = 0.0f;
      }

      communication.getCommand(serialNumber).setControlMode(controlMode.toByte());
      communication.getCommand(serialNumber).getGoalPositions()[index] = mappedValue;
      commandNotification.set();
   }
}
