package us.ihmc.rdx.ui.interactable;

import ihmc_psyonic_ros2.msg.dds.AbilityHandState;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.psyonicros2.AbilityHandController.ControlMode;
import us.ihmc.psyonicros2.AbilityHandHardwareCommunication;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.rdx.imgui.ImGuiTools;

import static us.ihmc.psyonicros2.AbilityHandInterface.ACTUATOR_COUNT;

public class RDXAbilityHand
{
   private RobotSide handSide;
   private final String serialNumber;

   private static final float OPEN_POSITION = 30.0f;
   private static final float CLOSED_POSITION = 90.0f;
   private static final float GRIP_POSITION = 100.0f;
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 120.0f;
   private static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiSliderFloat controlFingersSlider;
   private final ImGuiSliderFloat[] fingerSliders = new ImGuiSliderFloat[ACTUATOR_COUNT];

   private final AbilityHandHardwareCommunication communication;
   private AbilityHandState state;

   float[] currentPosition = new float[ACTUATOR_COUNT];
   float[] goalPosition = new float[ACTUATOR_COUNT];
   float[] goalVelocity = new float[ACTUATOR_COUNT];

   float[] actuatorPostions = new float[ACTUATOR_COUNT];


   public RDXAbilityHand(RobotSide handSide, String serialNumber, RDXBaseUI baseUI)
   {
      this.handSide = handSide;
      this.serialNumber = serialNumber;

      controlFingersSlider = new ImGuiSliderFloat("Control Fingers", "%.1f°", Float.NaN);
      controlFingersSlider.addWidgetAligner(widgetAligner);

      communication = new AbilityHandHardwareCommunication(handSide.toString());
      communication.start();

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         currentPosition[i] = OPEN_POSITION;
         goalPosition[i] = OPEN_POSITION;
         goalVelocity[i] = 0;
         String label = FINGER_NAMES[i];
         fingerSliders[i] = new ImGuiSliderFloat(label, "%.1f°", Float.NaN);
         fingerSliders[i].addWidgetAligner(widgetAligner);
         fingerSliders[i].setFloatValue(currentPosition[i]);
      }
   }

   public void update()
   {
      if(!communication.getAvailableHandSerialNumbers().contains(serialNumber))
         return;
      communication.publishCommand(serialNumber);
      state = communication.readState(serialNumber);
   }

   public void renderImGuiWidgets()
   {
      if(!communication.getAvailableHandSerialNumbers().contains(serialNumber))
         return;
      ImGui.pushID(handSide.ordinal());
      ImGui.sameLine();
      communication.getCommand(serialNumber).setSerialNumber(serialNumber);
      if (ImGui.button("Open"))
      {
         communication.getCommand(serialNumber).setControlMode(ControlMode.VEL_TO_POS.toByte());
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            communication.getCommand(serialNumber).getGoalPositions()[i] = OPEN_POSITION;
            communication.getCommand(serialNumber).getGoalVelocities()[i] = -30.0f;
         }
      }
      ImGui.sameLine();
      if (ImGui.button("Close"))
      {
         communication.getCommand(serialNumber).setControlMode(ControlMode.VEL_TO_POS.toByte());
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            communication.getCommand(serialNumber).getGoalPositions()[i] = CLOSED_POSITION;
            communication.getCommand(serialNumber).getGoalVelocities()[i] = 30.0f;
         }
      }
      ImGui.sameLine();
      if (ImGui.button("Grip"))
      {
         communication.getCommand(serialNumber).setControlMode(ControlMode.VEL_TO_POS.toByte());
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            communication.getCommand(serialNumber).getGoalPositions()[i] = GRIP_POSITION;
            communication.getCommand(serialNumber).getGoalVelocities()[i] = 30.0f;
         }
      }

      if(state != null)
      {
         actuatorPostions = state.getActuatorPositions();
      }

      float currentNotch = (actuatorPostions[0] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
      float sliderStart = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
      float sliderWidth = ImGui.getColumnWidth() - sliderStart;
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

      if (controlFingersSlider.render(SLIDER_MIN, SLIDER_MAX))
      {
         float newPos = controlFingersSlider.getFloatValue();
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            communication.getCommand(serialNumber).getGoalPositions()[i] = newPos;
         }
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
               communication.getCommand(serialNumber).setControlMode(ControlMode.POSITION.toByte());
               communication.getCommand(serialNumber).getGoalPositions()[i] = f;
            }
         }
      }
      ImGui.popID();
   }
}
