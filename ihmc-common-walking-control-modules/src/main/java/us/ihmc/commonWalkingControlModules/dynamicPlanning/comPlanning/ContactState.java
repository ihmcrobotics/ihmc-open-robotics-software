package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

public enum ContactState
{
   IN_CONTACT, FLIGHT;

   public boolean isLoadBearing()
   {
      switch (this)
      {
      case IN_CONTACT:
         return true;
      default:
         return false;
      }
   }
}
