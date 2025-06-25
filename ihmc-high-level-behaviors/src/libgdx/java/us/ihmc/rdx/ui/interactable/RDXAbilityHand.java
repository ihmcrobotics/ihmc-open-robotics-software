package us.ihmc.rdx.ui.interactable;

import imgui.ImGui;
import us.ihmc.psyonicros2.AbilityHandCommandType;
import us.ihmc.psyonicros2.AbilityHandInterface;
import us.ihmc.psyonicros2.AbilityHandHardwareCommunication;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXAbilityHand implements AbilityHandInterface
{
   private static final float OPEN_POSITION = 20.0f;
   private static final float CLOSED_POSITION = 80.0f;
   private static final float GRIP_POSITION = 90.0f;
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 120.0f;

   private final RobotSide handSide;
   private final AbilityHandHardwareCommunication communication;
   private final float[] commandValues = new float[ACTUATOR_COUNT];
   private final float[] actuatorPositions = new float[ACTUATOR_COUNT];
   private final float[] controlFingerSliders = new float[ACTUATOR_COUNT];

   private AbilityHandCommandType commandType = AbilityHandCommandType.POSITION;

   public RDXAbilityHand(RobotSide handSide, AbilityHandHardwareCommunication communication)
   {
      this.handSide = handSide;
      this.communication = communication;

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         commandValues[i] = 30.0f;
         actuatorPositions[i] = 30.0f;
         controlFingerSliders[i] = 30.0f;
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
      ImGui.pushID(handSide.ordinal());
      ImGui.text(handSide + ":");

      if (ImGui.button("Open"))
      {
         publishOnly(4, 20);
         try
         {
            Thread.sleep(400);
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException(e);
         }
         publishExcept(4, 20);
      }
      ImGui.sameLine();
      if (ImGui.button("Close"))
      {
         publishExcept(4, CLOSED_POSITION);
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException(e);
         }
         publishOnly(4,  60);
      }
      ImGui.sameLine();
      if(ImGui.button("Grip"))
      {
         setAllFingers(GRIP_POSITION);
         communication.publishCommand(this);
      }

      if (ImGui.sliderFloat("Control Fingers", controlFingerSliders, SLIDER_MIN, SLIDER_MAX, "%.1f°"))
      {
         setAllFingers(controlFingerSliders[0]);
         communication.publishCommand(this);
      }

      ImGui.beginDisabled();
      ImGui.sliderFloat("Current Angle", new float[] {actuatorPositions[0]}, SLIDER_MIN, SLIDER_MAX, "%.1f°");
      ImGui.endDisabled();

      ImGui.popID();
      ImGui.separator();
      ImGui.pushID(handSide.ordinal());
      if (ImGui.collapsingHeader("Individual Finger Control"))
      {
         ImGui.pushID("individual" + handSide.ordinal());
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            ImGui.pushID(i);
            float[] sliderVal = {controlFingerSliders[i]};
            if (ImGui.sliderFloat("Finger " + i, sliderVal, SLIDER_MIN, SLIDER_MAX, "%.1f°"))
            {
               controlFingerSliders[i] = sliderVal[0];
               float value = controlFingerSliders[i];
               setCommandType(AbilityHandCommandType.POSITION);
               if (i == 5)
               {
                  value = -value;
               }
               setCommandValue(i, value);
               communication.publishCommand(this);
            }
            ImGui.popID();
         }
         ImGui.popID();
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
            controlFingerSliders[i] = position;
            setCommandValue(i, position);
         }
      }
   }

   private void publishExcept(int excludedIndex, float position)
   {
      setCommandType(AbilityHandCommandType.POSITION);
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         if (i != excludedIndex)
         {
            float value = (i == 5) ? -position : position;
            controlFingerSliders[i] = position;
            setCommandValue(i, value);
         }
      }
      communication.publishCommand(this);
   }

   private void publishOnly(int index, float position)
   {
      setCommandType(AbilityHandCommandType.POSITION);
      controlFingerSliders[index] = position;
      setCommandValue(index, index == 5 ? -position : position);
      communication.publishCommand(this);
   }
}
