package us.ihmc.rdx.ui.interactable;

import us.ihmc.psyonicros2.AbilityHandCommandType;
import us.ihmc.psyonicros2.AbilityHandInterface;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXAbilityHand implements AbilityHandInterface
{
   private final RobotSide handSide;
   private AbilityHandCommandType commandType;
   private float[] commandValues = new float[ACTUATOR_COUNT];
   private float[] actuatorPositions = new float[ACTUATOR_COUNT];

   public RDXAbilityHand(RobotSide handSide)
   {
      this.handSide = handSide;
      commandType = AbilityHandCommandType.POSITION;
      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         commandValues[i] = 0;
         actuatorPositions[i] = 0;
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
   public void setCommandType(AbilityHandCommandType commandType)
   {
      this.commandType = commandType;
   }

   @Override
   public float getCommandValue(int index)
   {
      return commandValues[index];
   }

   @Override
   public void setCommandValue(int index, float value)
   {
      commandValues[index] = value;
   }

   @Override
   public float getActuatorPosition(int index)
   {
      return actuatorPositions[index];
   }

   @Override
   public void setActuatorPosition(int index, float value)
   {
      actuatorPositions[index] = value;
   }
}
