package us.ihmc.quadrupedRobotics.planning;

public enum ContactState
{
   IN_CONTACT, NO_CONTACT;

   public boolean isLoadingBearing()
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
