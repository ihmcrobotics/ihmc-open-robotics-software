package us.ihmc.commonWalkingControlModules.controlModules.reactiveBracing;

public enum ReactiveBracingStateEnum
{
   /* Moving towards contact point */
   PRE_CONTACT,
   /* Contact established, ramp up to nominal damping and loading */
   POST_CONTACT
}
