package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum PawPlannerType
{
   SIMPLE_PATH_TURN_WALK_TURN, VIS_GRAPH_WITH_TURN_WALK_TURN, A_STAR, VIS_GRAPH_WITH_A_STAR;

   public static final PawPlannerType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PawPlannerType generateRandomPlannerType(Random random)
   {
      return values[RandomNumbers.nextInt(random, 0, values.length - 1)];
   }

   public static PawPlannerType fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }

   public static PawPlannerType fromString(String name)
   {
      for (PawPlannerType value : values)
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
