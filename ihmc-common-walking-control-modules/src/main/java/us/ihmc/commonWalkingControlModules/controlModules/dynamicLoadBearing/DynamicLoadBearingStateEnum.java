package us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing;

public enum DynamicLoadBearingStateEnum
{
   /* Moving towards contact point */
   PRE_CONTACT,
   /* Contact established, ramp up to nominal damping and loading */
   POST_CONTACT
}
