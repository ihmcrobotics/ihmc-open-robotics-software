package us.ihmc.robotics.math.filters;

enum BacklashState
{
   BACKWARD_OK(false), FORWARD_OK(false), BACKWARD_SLOP(true), FORWARD_SLOP(true);

   private final boolean isInBacklash;

   private BacklashState(boolean inBacklash)
   {
      this.isInBacklash = inBacklash;
   }

   public boolean isInBacklash()
   {
      return isInBacklash;
   }
}
