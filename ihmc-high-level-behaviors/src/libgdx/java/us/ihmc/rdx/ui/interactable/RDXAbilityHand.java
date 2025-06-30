package us.ihmc.rdx.ui.interactable;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.psyonicros2.AbilityHandCommandType;
import us.ihmc.psyonicros2.AbilityHandController;
import us.ihmc.psyonicros2.AbilityHandHardwareCommunication;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.rdx.imgui.ImGuiTools;

import static us.ihmc.psyonicros2.AbilityHandInterface.ACTUATOR_COUNT;

public class RDXAbilityHand
{
   private static final float OPEN_POSITION = 30.0f;
   private static final float CLOSED_POSITION = 90.0f;
   private static final float GRIP_POSITION = 100.0f;
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 120.0f;
   private static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiSliderFloat controlFingersSlider;
   private final ImGuiSliderFloat[] fingerSliders = new ImGuiSliderFloat[ACTUATOR_COUNT];

   private final AbilityHandController controller;

   public RDXAbilityHand(RobotSide handSide, RDXBaseUI baseUI)
   {
      controller = new AbilityHandController(handSide);
      controlFingersSlider = new ImGuiSliderFloat("Control Fingers", "%.1f°", Float.NaN);
      controlFingersSlider.addWidgetAligner(widgetAligner);

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         String label = FINGER_NAMES[i];
         fingerSliders[i] = new ImGuiSliderFloat(label, "%.1f°", Float.NaN);
         fingerSliders[i].addWidgetAligner(widgetAligner);
         fingerSliders[i].setFloatValue(controller.getControlSliderValue(i));
      }
   }

   public void update(AbilityHandHardwareCommunication communication)
   {
      controller.update(communication);
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushID(controller.getHandSide().ordinal());
      ImGui.sameLine();
      if (ImGui.button("Open"))
      {
         controller.setGoalPosition(OPEN_POSITION);
         controller.setGoalVelocity(-30.0f);
         controller.setControlMode(1);
      }
      ImGui.sameLine();
      if (ImGui.button("Close"))
      {
         controller.setGoalPosition(CLOSED_POSITION);
         controller.setGoalVelocity(30.0f);
         controller.setControlMode(1);
      }
      ImGui.sameLine();
      if (ImGui.button("Grip"))
      {
         controller.setGoalPosition(GRIP_POSITION);
         controller.setGoalVelocity(30.0f);
         controller.setControlMode(1);
      }

      float currentNotch = (controller.getActuatorPosition(0) - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
      float sliderStart = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
      float sliderWidth = ImGui.getColumnWidth() - sliderStart;
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

      if (controlFingersSlider.render(SLIDER_MIN, SLIDER_MAX))
      {
         float newPos = controlFingersSlider.getFloatValue();
         controller.setAllFingers(newPos);
      }

      if (ImGui.collapsingHeader("Individual Finger Control"))
      {
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            float notchNorm = (i != 5) ?
                  (controller.getActuatorPosition(i) - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN) :
                  (-controller.getActuatorPosition(i) - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
            float startX = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
            float width = ImGui.getColumnWidth() - startX;
            ImGuiTools.renderSliderOrProgressNotch(startX + notchNorm * width, ImGui.getColorU32(ImGuiCol.Text));

            if (fingerSliders[i].render(SLIDER_MIN, SLIDER_MAX))
            {
               float val = fingerSliders[i].getFloatValue();
               controller.setControlSliderValue(i, val);
               controller.setCommandType(AbilityHandCommandType.POSITION);
               float f = (i == 5) ? -val : val;
               controller.setCommandValue(i, f);
            }
         }
      }
      ImGui.popID();
   }
}
