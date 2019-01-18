package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.geometry.transformables.OneDoFWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;

public class SimpleTrajectoryPoint1D implements OneDoFTrajectoryPointInterface
{
   private final OneDoFWaypoint oneDoFWaypoint = new OneDoFWaypoint();
   private final TrajectoryPoint trajectoryPoint = new TrajectoryPoint();

   public SimpleTrajectoryPoint1D()
   {
   }

   public SimpleTrajectoryPoint1D(double time, double position, double velocity)
   {
      set(time, position, velocity);
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
   public void setTime(double time)
   {
      trajectoryPoint.setTime(time);
   }

   @Override
   public double getTime()
   {
      return trajectoryPoint.getTime();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      return "Trajectory point 1D: (" + timeString + ", " + oneDoFWaypoint + ")";
   }
}
