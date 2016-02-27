package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPoint1DInterface;

public class SimpleTrajectoryPoint1D extends SimpleTrajectoryPoint<SimpleWaypoint1D, SimpleTrajectoryPoint1D>
      implements TrajectoryPoint1DInterface<SimpleTrajectoryPoint1D>
{
   public SimpleTrajectoryPoint1D()
   {
      super(new SimpleWaypoint1D());
   }

   public void setPosition(double position)
   {
      waypointData.setPosition(position);
   }

   public void setVelocity(double velocity)
   {
      waypointData.setVelocity(velocity);
   }

   public void set(double time, double position, double velocity)
   {
      setTime(time);
      waypointData.set(position, velocity);
   }

   public void set(TrajectoryPoint1DInterface<?> other)
   {
      setTime(other.getTime());
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
   }

   @Override
   public double getPosition()
   {
      return waypointData.getPosition();
   }

   @Override
   public double getVelocity()
   {
      return waypointData.getPosition();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      return "Trajectory point 1D: (" + timeString + ", " + waypointData + ")";
   }
}
