package us.ihmc.rdx.ui.hands.psyonicAbilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2HardwareCommunication;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.hands.RDXHandInterface;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Arrays;

import static us.ihmc.handsros2.abilityHand.AbilityHandInterface.ACTUATOR_COUNT;

public class RDXAbilityHand implements RDXHandInterface
{
   private static final float START_POSITION = 30.0f;
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 120.0f;
   private static final float GRIP_VELOCITY = 30.0f;
   private static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final String identifier;
   private final AbilityHandROS2HardwareCommunication communication;
   private final RobotSide handSide;

   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiSliderFloat controlFingersSlider;
   private final ImGuiSliderFloat[] fingerSliders = new ImGuiSliderFloat[ACTUATOR_COUNT];

   private final Notification commandNotification = new Notification();
   private final Throttler publishThrottler = new Throttler();

   private ControlMode controlMode;

   private float[] actuatorPositions = new float[ACTUATOR_COUNT];

   private ControlMode previousControl = null;

   public RDXAbilityHand(String identifier, RobotSide handSide, AbilityHandROS2HardwareCommunication communication)
   {
      this.identifier = identifier;
      this.handSide = handSide;
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
      }

      publishThrottler.setFrequency(30.0);
   }

   @Override
   public void update()
   {
      if (!communication.getAvailableHands().contains(identifier))
         return;

      actuatorPositions = communication.readState(identifier).getActuatorPositions();

      if(publishThrottler.run() && commandNotification.poll())
      {
         publishCommand();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (!communication.getAvailableHands().contains(identifier))
         return;

      ImGui.pushID(identifier);
      ImGui.text(getSide().toString() + " Ability Hand");

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

      float currentNotch = (actuatorPositions[0] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
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
         communication.getCommand(identifier).setControlMode(controlMode.toByte());
         for (int i = 0; i < ACTUATOR_COUNT - 2; i++)
         {
            communication.getCommand(identifier).getGoalPositions()[i] = newPos;
         }
         commandNotification.set();
      }

      if (ImGui.collapsingHeader("Individual Finger Control"))
      {
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            float notchNorm = (i != 5) ?
                  (actuatorPositions[i] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN) :
                  (-actuatorPositions[i] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
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
               communication.getCommand(identifier).setControlMode(controlMode.toByte());
               communication.getCommand(identifier).getGoalPositions()[i] = f;
               commandNotification.set();
            }
         }
      }

      ImGui.popID();
   }

   private void gripMode(Grip grip, AbilityHandROS2HardwareCommunication communication)
   {
      if (!communication.getAvailableHands().contains(identifier))
         return;

      Arrays.fill(communication.getCommand(identifier).getGoalVelocities(), GRIP_VELOCITY);
      communication.getCommand(identifier).setControlMode(ControlMode.GRIP.toByte());
      communication.getCommand(identifier).setGrip(grip.toByte());

      commandNotification.set();
      previousControl = ControlMode.GRIP;
   }

   private synchronized void publishCommand()
   {
      if (!communication.getAvailableHands().contains(identifier))
         return;

      communication.publishCommand(identifier);
   }

   private void syncPositionSliders()
   {
      if (!communication.getAvailableHands().contains(identifier))
         return;

      float[] cmdPos = communication.getCommand(identifier).getGoalPositions();
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         float live = actuatorPositions[i];
         cmdPos[i] = live;
         float val = (i == 5) ? -live : live;
         fingerSliders[i].setFloatValue(val);
      }
      controlFingersSlider.setFloatValue(cmdPos[0]);
   }

   @Override
   public String getIdentifier()
   {
      return identifier;
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
      if (!communication.getAvailableHands().contains(identifier))
         return;

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

      Arrays.fill(communication.getCommand(identifier).getGoalVelocities(), GRIP_VELOCITY);
      communication.getCommand(identifier).setControlMode(AbilityHandCommand.GRIP_CONTROL);
      communication.getCommand(identifier).setGrip(abilityHandGrip);
      previousControl = ControlMode.GRIP;
      publishCommand();
   }
}
