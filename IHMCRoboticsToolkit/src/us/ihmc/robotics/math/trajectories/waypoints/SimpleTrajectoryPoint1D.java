package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.geometry.transformables.OneDoFWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;

public class SimpleTrajectoryPoint1D extends SimpleTrajectoryPoint<OneDoFWaypoint, SimpleTrajectoryPoint1D>
      implements OneDoFTrajectoryPointInterface<SimpleTrajectoryPoint1D>
{
   public SimpleTrajectoryPoint1D()
   {
      super(new OneDoFWaypoint());
   }
   
   public SimpleTrajectoryPoint1D(double time, double position, double velocity)
   {
      this();
      set(time, position, velocity);
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

   public void set(OneDoFTrajectoryPointInterface<?> other)
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
      return waypointData.getVelocity();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      return "Trajectory point 1D: (" + timeString + ", " + waypointData + ")";
   }
}
