package us.ihmc.rdx.ui.interactable;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.psyonicros2.AbilityHandCommandType;
import us.ihmc.psyonicros2.AbilityHandInterface;
import us.ihmc.psyonicros2.AbilityHandHardwareCommunication;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.rdx.imgui.ImGuiTools;

public class RDXAbilityHand implements AbilityHandInterface
{
   private static final float OPEN_POSITION = 30.0f;
   private static final float CLOSED_POSITION = 90.0f;
   private static final float GRIP_POSITION = 100.0f;
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 120.0f;
   private static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};
   private final RobotSide handSide;
   private final AbilityHandHardwareCommunication communication;
   private final float[] commandValues = new float[ACTUATOR_COUNT];
   private final float[] actuatorPositions = new float[ACTUATOR_COUNT];
   private final float[] controlFinger = new float[ACTUATOR_COUNT];

   private enum ControlMode
   {IDLE, VEL_TO_POS}

   private ControlMode controlMode = ControlMode.IDLE;
   private float velToPosTarget = Float.NaN;
   private float velToPosSpeed = 0.0f;

   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();

   private final String controlFingersSliderLabel;
   private final ImGuiSliderFloat controlFingersSlider;

   private final ImGuiSliderFloat[] fingerSliders = new ImGuiSliderFloat[ACTUATOR_COUNT];

   private AbilityHandCommandType commandType = AbilityHandCommandType.POSITION;

   public RDXAbilityHand(RobotSide handSide, AbilityHandHardwareCommunication communication, RDXBaseUI baseUI)
   {
      this.handSide = handSide;
      this.communication = communication;
      controlFingersSliderLabel = "Control Fingers";
      controlFingersSlider = new ImGuiSliderFloat(controlFingersSliderLabel, "%.1f°", Float.NaN);
      controlFingersSlider.addWidgetAligner(widgetAligner);

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         commandValues[i] = 30.0f;
         actuatorPositions[i] = 30.0f;
         controlFinger[i] = 30.0f;

         String label = FINGER_NAMES[i];
         fingerSliders[i] = new ImGuiSliderFloat(label, "%.1f°", Float.NaN);
         fingerSliders[i].addWidgetAligner(widgetAligner);
         fingerSliders[i].setFloatValue(controlFinger[i]);
      }
   }

   @Override
   public RobotSide getHandSide()
   {
      return handSide;
   }

   @Override
   public AbilityHandCommandType getCommandType()
   {
      return commandType;
   }

   @Override
   public void setCommandType(AbilityHandCommandType type)
   {
      this.commandType = type;
   }

   @Override
   public float getCommandValue(int index)
   {
      return commandValues[index];
   }

   @Override
   public void setCommandValue(int index, float v)
   {
      commandValues[index] = v;
   }

   @Override
   public float getActuatorPosition(int index)
   {
      return actuatorPositions[index];
   }

   @Override
   public void setActuatorPosition(int index, float v)
   {
      actuatorPositions[index] = v;
   }

   public void update()
   {
      communication.readState(this);
   }

   public void renderImGuiWidgets()
   {
      processVelToPos(4);
      ImGui.pushID(handSide.ordinal());
      ImGui.text(handSide + ":");
      ImGui.sameLine();
      if (ImGui.button("Open"))
      {
         startVelToPos(OPEN_POSITION, -30.0f);
      }
      ImGui.sameLine();
      if (ImGui.button("Close"))
      {
         startVelToPos(CLOSED_POSITION, 30.0f);
      }
      ImGui.sameLine();
      if (ImGui.button("Grip"))
      {
         startVelToPos(GRIP_POSITION, 30.0f);
      }

      float currentNotch = (actuatorPositions[0] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
      float sliderStart = widgetAligner.getCursorMaxX() + ImGui.getStyle().getItemSpacingX();
      float sliderWidth = ImGui.getColumnWidth() - sliderStart;
      ImGuiTools.renderSliderOrProgressNotch(sliderStart + currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

      if (controlFingersSlider.render(SLIDER_MIN, SLIDER_MAX))
      {
         float newPos = controlFingersSlider.getFloatValue();
         setAllFingers(newPos);
         communication.publishCommand(this);
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
               float val = fingerSliders[i].getFloatValue();
               controlFinger[i] = val;
               setCommandType(AbilityHandCommandType.POSITION);
               float f = (i == 5) ? -val : val;
               setCommandValue(i, f);
               communication.publishCommand(this);
            }
         }
      }
      ImGui.popID();
   }

   private void setAllFingers(float position)
   {
      setCommandType(AbilityHandCommandType.POSITION);
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         if (i != 4 && i != 5)
         {
            controlFinger[i] = position;
            setCommandValue(i, position);
         }
      }
   }

   private void startVelToPos(float targetPosition, float velocity)
   {
      velToPosTarget = targetPosition;
      velToPosSpeed = velocity;
      controlMode = ControlMode.VEL_TO_POS;
   }

   private void processVelToPos(int excludedIndex)
   {
      if (controlMode != ControlMode.VEL_TO_POS)
         return;

      if (velToPosSpeed < 0.0f)
      {
         if (moveSkipped(excludedIndex))
         {
            setCommandType(AbilityHandCommandType.VELOCITY);
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
               if (i != excludedIndex)
               {
                  float v = (i == 5) ? -velToPosSpeed : velToPosSpeed;
                  setCommandValue(i, v);
               }
               else
               {
                  setCommandValue(i, 0.0f);
               }
            }
            communication.publishCommand(this);
            update();

            boolean allOthersReached = true;
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
               if (i == excludedIndex)
                  continue;
               float pos = getActuatorPosition(i);
               if (i == 5)
                  pos = -pos;
               if ((velToPosSpeed < 0 && pos > velToPosTarget) || (velToPosSpeed > 0 && pos < velToPosTarget))
               {
                  allOthersReached = false;
                  break;
               }
            }
            if (allOthersReached)
               controlMode = ControlMode.IDLE;
         }
      }
      else
      {
         setCommandType(AbilityHandCommandType.VELOCITY);
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            if (i != excludedIndex)
            {
               float v = (i == 5) ? -velToPosSpeed : velToPosSpeed;
               setCommandValue(i, v);
            }
            else
            {
               setCommandValue(i, 0.0f);
            }
         }
         communication.publishCommand(this);
         update();

         boolean reached = true;
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            if (i == excludedIndex)
               continue;
            float pos = getActuatorPosition(i);
            if (i == 5)
               pos = -pos;
            if ((velToPosSpeed > 0 && pos < velToPosTarget) || (velToPosSpeed < 0 && pos > velToPosTarget))
            {
               reached = false;
               break;
            }
         }
         if (reached)
         {
            if (moveSkipped(excludedIndex))
               controlMode = ControlMode.IDLE;
         }
      }
   }

   private boolean moveSkipped(int index)
   {
      setCommandType(AbilityHandCommandType.VELOCITY);
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         if (i == index)
         {
            float v = (i == 5) ? -velToPosSpeed : velToPosSpeed;
            setCommandValue(i, v);
         }
         else
         {
            setCommandValue(i, 0);
         }
      }
      communication.publishCommand(this);
      update();
      boolean reached = true;
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         if (i == index)
         {
            float pos = getActuatorPosition(i);
            if ((velToPosSpeed > 0 && pos < velToPosTarget / 2) || (velToPosSpeed < 0 && pos > velToPosTarget))
            {
               reached = false;
               break;
            }
         }
      }
      if (reached)
      {
         return true;
      }
      return false;
   }
}
