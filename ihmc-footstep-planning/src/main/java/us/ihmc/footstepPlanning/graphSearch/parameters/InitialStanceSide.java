package us.ihmc.footstepPlanning.graphSearch.parameters;

public enum InitialStanceSide
{
   /** Use stance foot that's closest to the goal. */
   AUTO,
   LEFT,
   RIGHT;

   public static final InitialStanceSide[] values = values();

   public static InitialStanceSide fromByte(byte initialStanceSide)
   {
      return switch(initialStanceSide)
      {
         case 0 -> LEFT;
         case 1 -> RIGHT;
         default -> AUTO;
      };
   }

   public byte toByte()
   {
      return switch (this)
      {
         case LEFT -> 0;
         case RIGHT -> 1;
         case AUTO -> 2;
      };
   }
}
