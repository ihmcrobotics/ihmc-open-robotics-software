package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum FootstepPlannerType
{
   SIMPLE_PATH_TURN_WALK_TURN, VIS_GRAPH_WITH_TURN_WALK_TURN, A_STAR, VIS_GRAPH_WITH_A_STAR;

   public static final FootstepPlannerType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlannerType generateRandomPlannerType(Random random)
   {
      return values[RandomNumbers.nextInt(random, 0, values.length - 1)];
   }

   public static FootstepPlannerType fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }

   public static FootstepPlannerType fromString(String name)
   {
      for (FootstepPlannerType value : values)
      {
         if (name.equals(value.name()))
            return value;
      }

      throw new IllegalArgumentException("Unable to find a corresponding FootstepPlannerType to string : " + name);
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
