package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;

public class SoleWaypoint
{
   private Point3D position;
   private Vector3D velocity;
   private double time;

   public SoleWaypoint()
   {
      position = new Point3D(0, 0, 0);
      velocity = new Vector3D(0, 0, 0);
      time = 0.0;
   }

   public SoleWaypoint(SoleWaypoint other)
   {
      this();
      set(other);
   }

   public SoleWaypoint(Point3D position, Vector3D velocity, double time)
   {
      this();
      set(position, velocity, time);
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Vector3D getVelocity()
   {
      return velocity;
   }

   public void getPosition(Point3D position)
   {
      position.set(this.position);
   }

   public void getVelocity(Vector3D velocity)
   {
      velocity.set(this.velocity);
   }

   public double getTime()
   {
      return time;
   }

   public void set(Point3D position, Vector3D velocity, Double time)
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

   public void setPosition(Point3D position)
   {
      this.position.set(position);
   }

   public void setVelocity(Vector3D velocity)
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
