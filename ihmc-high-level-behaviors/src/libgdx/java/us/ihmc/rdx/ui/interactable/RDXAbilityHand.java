// RDXAbilityHand.java
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
   private static final float SLIDER_MIN = 0.0f;
   private static final float SLIDER_MAX = 90.0f;

   private final RobotSide handSide;
   private final AbilityHandHardwareCommunication communication;
   private final float[] commandValues = new float[ACTUATOR_COUNT];
   private final float[] actuatorPositions = new float[ACTUATOR_COUNT];
   private final float[] sliderValue = new float[] {0.0f};

   private AbilityHandCommandType commandType = AbilityHandCommandType.POSITION;

   public RDXAbilityHand(RobotSide handSide, AbilityHandHardwareCommunication communication)
   {
      this.handSide = handSide;
      this.communication = communication;

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         commandValues[i] = 0.0f;
         actuatorPositions[i] = 0.0f;
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
         setAllFingers(OPEN_POSITION);
         communication.publishCommand(this);
      }
      ImGui.sameLine();
      if (ImGui.button("Close"))
      {
         setAllFingers(CLOSED_POSITION);
         communication.publishCommand(this);
      }

      if (ImGui.sliderFloat("Control Fingers", sliderValue, SLIDER_MIN, SLIDER_MAX))
      {
         setAllFingers(sliderValue[0]);
         communication.publishCommand(this);
      }

      ImGui.popID();
      ImGui.separator();
   }

   private void setAllFingers(float position)
   {
      setCommandType(AbilityHandCommandType.POSITION);
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         if (i != 4 && i != 5)
            setCommandValue(i, position);
      }
   }
}
