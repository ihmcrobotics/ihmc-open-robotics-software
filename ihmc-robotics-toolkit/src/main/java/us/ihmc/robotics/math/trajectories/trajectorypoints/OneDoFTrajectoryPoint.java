package us.ihmc.robotics.math.trajectories.trajectorypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.OneDoFWaypoint;

public class OneDoFTrajectoryPoint implements OneDoFTrajectoryPointBasics
{
   private final OneDoFWaypoint oneDoFWaypoint = new OneDoFWaypoint();
   private double time;

   public OneDoFTrajectoryPoint()
   {
   }

   public OneDoFTrajectoryPoint(double time, double position, double velocity, double acceleration)
   {
      set(time, position, velocity, acceleration);
   }

   @Override
   public void setPosition(double position)
   {
      oneDoFWaypoint.setPosition(position);
   }

   @Override
   public void setVelocity(double velocity)
   {
      oneDoFWaypoint.setVelocity(velocity);
   }

   @Override
   public void setAcceleration(double acceleration)
   {
      oneDoFWaypoint.setAcceleration(acceleration);
   }

   @Override
   public double getPosition()
   {
      return oneDoFWaypoint.getPosition();
   }

   @Override
   public double getVelocity()
   {
      return oneDoFWaypoint.getVelocity();
   }

   @Override
   public double getAcceleration()
   {
      return oneDoFWaypoint.getAcceleration();
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
      return "Trajectory point 1D: (" + timeString + ", " + oneDoFWaypoint + ")";
   }
}
