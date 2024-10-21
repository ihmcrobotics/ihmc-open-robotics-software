package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3PIDGainsWaypointBasics;

public class SE3PIDGainsWaypoint implements SE3PIDGainsWaypointBasics
{
   private PID3DGains linearGains;
   private PID3DGains angularGains;

   @Override
   public void setAngular(PID3DGains angular)
   {
      this.angularGains = angular;
   }

   @Override
   public void setLinear(PID3DGains linear)
   {
      this.linearGains = linear;
   }

   @Override
   public PID3DGains getAngular()
   {
      return angularGains;
   }

   @Override
   public PID3DGains getLinear()
   {
      return linearGains;
   }

   @Override
   public String toString()
   {
      String linearString = "linear = " + linearGains.toString();
      String angularString = "angular = " + angularGains.toString();
      return "PIDGainsWaypoint: (" + linearString + ", " + angularString + ")";
   }
}
