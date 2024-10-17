package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3PIDGainsTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.SE3PIDGainsWaypoint;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class SE3PIDGainsTrajectoryPoint implements SE3PIDGainsTrajectoryPointBasics
{
   private final SE3PIDGainsWaypoint gainsWaypoint = new SE3PIDGainsWaypoint();
   private double time;

   public SE3PIDGainsTrajectoryPoint()
   {
   }

   public SE3PIDGainsTrajectoryPoint(double time, PID3DGains angularGains, PID3DGains linearGains)
   {
      set(time, angularGains, linearGains);
   }

   @Override
   public void setAngular(PID3DGains angular)
   {
      gainsWaypoint.setAngular(angular);
   }

   @Override
   public void setLinear(PID3DGains linear)
   {
      gainsWaypoint.setLinear(linear);
   }

   @Override
   public PID3DGains getAngular()
   {
      return gainsWaypoint.getAngular();
   }

   @Override
   public PID3DGains getLinear()
   {
      return gainsWaypoint.getLinear();
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      return "PIDGainsTrajectoryPoint: (" + timeString + ", " + gainsWaypoint.toString() + ")";
   }
}
