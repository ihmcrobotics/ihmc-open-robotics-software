package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;

public class TorusPosePacket extends Packet<TorusPosePacket> implements TransformableDataObject<TorusPosePacket>
{
   public Point3D position;
   public Quaternion orientation;
   public double fingerHoleRadius;

   public int index = 0;

   public TorusPosePacket()
   {
      // Empty constructor for deserialization
   }

   public TorusPosePacket(Point3D position, Quaternion orientation, double fingerHoleRadius)
   {
      this.orientation = orientation;
      this.position = position;
      this.fingerHoleRadius = fingerHoleRadius;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public Point3D getPosition()
   {
      return position;
   }

   public double getFingerHoleRadius()
   {
      return fingerHoleRadius;
   }

   public TorusPosePacket transform(RigidBodyTransform transform)
   {
      TorusPosePacket ret = new TorusPosePacket();

      ret.fingerHoleRadius = this.fingerHoleRadius;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);
      ret.position = TransformTools.getTransformedPoint(this.getPosition(), transform);

      return ret;
   }

   public String toString()
   {
      return this.getClass().getSimpleName();
   }

   public boolean epsilonEquals(TorusPosePacket other, double epsilon)
   {
      boolean ret;
      final double thisRadius = this.getFingerHoleRadius();
      final double otherRadius = other.getFingerHoleRadius();
      final double delta = Math.abs(thisRadius - otherRadius);
      if (thisRadius == otherRadius)
         ret = true;
      else if ((thisRadius == 0) || (otherRadius == 0) || (delta < 1e-2))
         ret = (delta < epsilon);
      else
      {
         ret = (delta / (thisRadius + otherRadius) < epsilon);
      }
      ret &= RotationTools.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      ret &= this.getPosition().epsilonEquals(other.getPosition(), epsilon);

      return ret;
   }

   public TorusPosePacket(Random random)
   {
      Point3D point = new Point3D();
      Quaternion quat = new Quaternion();

      point.set(RandomTools.generateRandomPoint(random, 0.288, 0.288, 0.288));    // magic numbers so point will not exceed XYZ_MIN / MAX in TorusPosePacketSerializer
      quat.set(RandomTools.generateRandomRotation(random));

      this.orientation = quat;
      this.position = point;
      this.fingerHoleRadius = random.nextDouble();
   }
}
