package us.ihmc.footstepPlanning;

public enum FootstepPlanHeading
{
   FORWARD,
   BACKWARD,
   LEFT,
   RIGHT;

   public double getYawOffset()
   {
      switch(this)
      {
         case BACKWARD:
            return Math.PI;
         case LEFT:
            return -0.5 * Math.PI;
         case RIGHT:
            return 0.5 * Math.PI;
         case FORWARD:
         default:
            return 0.0;
      }
   }

   public static FootstepPlanHeading fromByte(byte index)
   {
      if (index == -1)
         return null;
      else
         return values()[index];
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }
}
