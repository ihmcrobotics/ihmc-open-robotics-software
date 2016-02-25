package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class SimpleWaypoint1D implements Waypoint1DInterface
{
   private double time;
   private double position;
   private double velocity;

   public void setTime(double time)
   {
      this.time = time;
   }

   public void setPosition(double position)
   {
      this.position = position;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   public void set(double time, double position, double velocity)
   {
      setTime(time);
      setPosition(position);
      setVelocity(velocity);
   }

   public void set(Waypoint1DInterface other)
   {
      setTime(other.getTime());
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public double getPosition()
   {
      return position;
   }

   @Override
   public double getVelocity()
   {
      return velocity;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      return "(time = " + doubleFormat.format(time) + ", position = " + doubleFormat.format(position) + ", velocity = " + doubleFormat.format(velocity) + ")";
   }
}
