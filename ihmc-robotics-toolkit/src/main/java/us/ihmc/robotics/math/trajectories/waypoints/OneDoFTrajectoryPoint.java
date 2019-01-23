package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.geometry.transformables.OneDoFWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;

public class OneDoFTrajectoryPoint implements OneDoFTrajectoryPointInterface
{
   private final OneDoFWaypoint oneDoFWaypoint = new OneDoFWaypoint();
   private double time;

   public OneDoFTrajectoryPoint()
   {
   }

   public OneDoFTrajectoryPoint(double time, double position, double velocity)
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
