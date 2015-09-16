package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;

public class TorusPosePacket extends Packet<TorusPosePacket> implements TransformableDataObject<TorusPosePacket>
{
   public Point3d position;
   public Quat4d orientation;
   public double fingerHoleRadius;

   public int index = 0;

   public TorusPosePacket()
   {
      // Empty constructor for deserialization
   }

   public TorusPosePacket(Point3d position, Quat4d orientation, double fingerHoleRadius)
   {
      this.orientation = orientation;
      this.position = position;
      this.fingerHoleRadius = fingerHoleRadius;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public Point3d getPosition()
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
      ret &= RotationFunctions.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      ret &= this.getPosition().epsilonEquals(other.getPosition(), epsilon);

      return ret;
   }

   public TorusPosePacket(Random random)
   {
      Point3d point = new Point3d();
      Quat4d quat = new Quat4d();

      point.set(RandomTools.generateRandomPoint(random, 0.288, 0.288, 0.288));    // magic numbers so point will not exceed XYZ_MIN / MAX in TorusPosePacketSerializer
      quat.set(RandomTools.generateRandomRotation(random));

      this.orientation = quat;
      this.position = point;
      this.fingerHoleRadius = random.nextDouble();
   }
}
