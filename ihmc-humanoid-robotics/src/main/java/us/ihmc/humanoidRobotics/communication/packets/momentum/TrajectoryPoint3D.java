package us.ihmc.humanoidRobotics.communication.packets.momentum;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class TrajectoryPoint3D implements Settable<TrajectoryPoint3D>, us.ihmc.robotics.lists.Settable<TrajectoryPoint3D>, EpsilonComparable<TrajectoryPoint3D>
{
   /** Position of trajectory point */
   public final Point3D position;

   /** Velocity of trajectory point */
   public final Vector3D velocity;

   /** Time of trajectory point */
   public double time;

   public TrajectoryPoint3D(Tuple3DReadOnly position, Tuple3DReadOnly veclocity, double time)
   {
      this.position = new Point3D(position);
      this.velocity = new Vector3D(veclocity);
      this.time = time;
   }

   public TrajectoryPoint3D(FramePoint3D position, FrameVector3D velocity, double time)
   {
      this.time = time;
      this.position = new Point3D(position);
      this.velocity = new Vector3D(velocity);
   }

   public TrajectoryPoint3D()
   {
      this.position = new Point3D();
      this.velocity = new Vector3D();
      this.time = 0.0;
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setVelocity(Tuple3DReadOnly velocity)
   {
      this.velocity.set(velocity);
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public Tuple3DReadOnly getPosition()
   {
      return position;
   }

   public Tuple3DReadOnly getVelocity()
   {
      return velocity;
   }

   public double getTime()
   {
      return time;
   }

   @Override
   public void set(TrajectoryPoint3D other)
   {
      setPosition(other.getPosition());
      setVelocity(other.getVelocity());
      setTime(other.getTime());
   }

   @Override
   public boolean epsilonEquals(TrajectoryPoint3D other, double epsilon)
   {
      if (!getPosition().epsilonEquals(other.getPosition(), epsilon))
      {
         return false;
      }
      if (!getVelocity().epsilonEquals(other.getVelocity(), epsilon))
      {
         return false;
      }
      if (!EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon))
      {
         return false;
      }
      return true;
   }

}
