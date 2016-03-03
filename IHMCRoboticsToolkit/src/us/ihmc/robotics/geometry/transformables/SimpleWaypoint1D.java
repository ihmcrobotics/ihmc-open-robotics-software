package us.ihmc.robotics.geometry.transformables;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.Waypoint1DInterface;

public class SimpleWaypoint1D implements Waypoint1DInterface<SimpleWaypoint1D>
{
   private double position;
   private double velocity;

   public SimpleWaypoint1D()
   {
   }

   @Override
   public void setPosition(double position)
   {
      this.position = position;
   }

   @Override
   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   public void set(double position, double velocity)
   {
      this.position = position;
      this.velocity = velocity;
   }

   @Override
   public void set(SimpleWaypoint1D other)
   {
      position = other.position;
      velocity = other.velocity;
   }

   @Override
   public void setToZero()
   {
      position = 0.0;
      velocity = 0.0;
   }

   @Override
   public void setToNaN()
   {
      position = Double.NaN;
      velocity = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(position) || Double.isNaN(velocity);
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
      String positionString = "position = " + doubleFormat.format(getPosition());
      String velocityString = "velocity = " + doubleFormat.format(getVelocity());
      return "Waypoint 1D: (" + positionString + ", " + velocityString + ")";
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      // Do nothing since they are just numbers here.
   }
}
