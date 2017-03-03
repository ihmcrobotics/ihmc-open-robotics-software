package us.ihmc.wholeBodyController;

public enum DRCHandType
{
   NONE(true), IROBOT(false), HOOK(false), ROBOTIQ(true), VALKYRIE(false), ROBOTIQ_AND_SRI(false);

   private final boolean simulateHand;

   private DRCHandType(boolean simulateHand)
   {
      this.simulateHand = simulateHand;
   }

   public boolean isHandSimulated()
   {
      return simulateHand;
   }
}
