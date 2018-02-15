package us.ihmc.quadrupedRobotics.controlModules.foot;

public enum QuadrupedFootStates
{
   SUPPORT,
   SWING,
   MOVE_VIA_WAYPOINTS,
   HOLD;

   public static final QuadrupedFootStates[] values = values();
}
