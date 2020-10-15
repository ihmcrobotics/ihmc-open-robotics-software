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
            return 0.5 * Math.PI;
         case RIGHT:
            return -0.5 * Math.PI;
         case FORWARD:
         default:
            return 0.0;
      }
   }
}
