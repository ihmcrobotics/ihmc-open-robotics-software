package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.MathTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class SoleWaypoint
{
   private Point3d position;
   private Vector3d velocity;
   private double time;

   public SoleWaypoint()
   {
      position = new Point3d(0, 0, 0);
      velocity = new Vector3d(0, 0, 0);
      time = 0.0;
   }

   public SoleWaypoint(SoleWaypoint other)
   {
      this();
      set(other);
   }

   public SoleWaypoint(Point3d position, Vector3d velocity, double time)
   {
      this();
      set(position, velocity, time);
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Vector3d getVelocity()
   {
      return velocity;
   }

   public void getPosition(Point3d position)
   {
      position.set(this.position);
   }

   public void getVelocity(Vector3d velocity)
   {
      velocity.set(this.velocity);
   }

   public double getTime()
   {
      return time;
   }

   public void set(Point3d position, Vector3d velocity, Double time)
   {
      this.position.set(position);
      this.velocity.set(velocity);
      this.time = time;
   }

   public void set(SoleWaypoint other)
   {
      this.position.set(other.position);
      this.velocity.set(other.velocity);
      this.time = other.time;
   }

   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   public void setVelocity(Vector3d velocity)
   {
      this.velocity.set(velocity);
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public boolean epsilonEquals(SoleWaypoint other, double epsilon)
   {
      return position.epsilonEquals(other.position, epsilon) || velocity.epsilonEquals(other.velocity, epsilon) || MathTools
            .epsilonEquals(time, other.time, epsilon);
   }
}
