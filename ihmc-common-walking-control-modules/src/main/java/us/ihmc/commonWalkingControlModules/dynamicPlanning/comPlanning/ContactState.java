package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

/**
 * This defines the contact state for the contact sequence used by the {@link CoMTrajectoryPlanner}. It is either in contact, or in flight.
 */
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
