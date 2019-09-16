package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum PawStepPlannerType
{
   SIMPLE_PATH_TURN_WALK_TURN, VIS_GRAPH_WITH_TURN_WALK_TURN, A_STAR, VIS_GRAPH_WITH_A_STAR;

   public static final PawStepPlannerType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PawStepPlannerType generateRandomPlannerType(Random random)
   {
      return values[RandomNumbers.nextInt(random, 0, values.length - 1)];
   }

   public static PawStepPlannerType fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }

   public static PawStepPlannerType fromString(String name)
   {
      for (PawStepPlannerType value : values)
      {
         if (name.equals(value.name()))
            return value;
      }

      throw new IllegalArgumentException("Unable to find a corresponding PawPlannerType to string : " + name);
   }

   public boolean plansPath()
   {
      switch (this)
      {
      case SIMPLE_PATH_TURN_WALK_TURN:
      case VIS_GRAPH_WITH_TURN_WALK_TURN:
         return true;
      default:
         return false;
      }
   }

}
