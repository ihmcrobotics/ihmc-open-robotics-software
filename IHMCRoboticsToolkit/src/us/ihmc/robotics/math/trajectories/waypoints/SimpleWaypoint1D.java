package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.MathTools;

public class SimpleWaypoint1D implements Waypoint1DInterface<SimpleWaypoint1D>
{
   private double time;
   private double position;
   private double velocity;

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      time += timeOffsetToAdd;
   }

   @Override
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
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

   public void set(Waypoint1DInterface<?> other)
   {
      setTime(other.getTime());
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
   }

   @Override
   public void set(SimpleWaypoint1D waypoint)
   {
      time = waypoint.time;
      position = waypoint.position;
      velocity = waypoint.velocity;
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
   public boolean epsilonEquals(SimpleWaypoint1D other, double epsilon)
   {
      if (!MathTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!MathTools.epsilonEquals(getPosition(), other.getPosition(), epsilon))
         return false;
      if (!MathTools.epsilonEquals(getVelocity(), other.getVelocity(), epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      String positionString = "position = " + doubleFormat.format(getPosition());
      String velocityString = "velocity = " + doubleFormat.format(getVelocity());
      return "(" + timeString + ", " + positionString + ", " + velocityString + ")";
   }
}
