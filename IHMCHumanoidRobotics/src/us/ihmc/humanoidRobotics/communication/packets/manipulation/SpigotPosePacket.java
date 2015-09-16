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

public class SpigotPosePacket extends Packet<SpigotPosePacket> implements TransformableDataObject<SpigotPosePacket>
{
   public Point3d position;
   public Quat4d orientation;

   public int index = 0;

   public SpigotPosePacket()
   {
      // Empty constructor for deserialization
   }

   public SpigotPosePacket(Point3d position, Quat4d orientation)
   {
      this.orientation = orientation;
      this.position = position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public SpigotPosePacket transform(RigidBodyTransform transform)
   {
      SpigotPosePacket ret = new SpigotPosePacket();

      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);
      ret.position = TransformTools.getTransformedPoint(this.getPosition(), transform);

      return ret;
   }

   public String toString()
   {
      return this.getClass().getSimpleName();
   }

   public boolean epsilonEquals(SpigotPosePacket other, double epsilon)
   {
      boolean ret = RotationFunctions.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      ret &= this.getPosition().epsilonEquals(other.getPosition(), epsilon);

      return ret;
   }

   public SpigotPosePacket(Random random)
   {
      Point3d point = new Point3d();
      Quat4d quat = new Quat4d();

      point.set(RandomTools.generateRandomPoint(random, 0.288, 0.288, 0.288));    // magic

      // numbers
      // so
      // point
      // will
      // not
      // exceed
      // XYZ_MIN
      // /
      // MAX
      // in
      // TorusPosePacketSerializer
      quat.set(RandomTools.generateRandomRotation(random));

      this.position = point;
      this.orientation = quat;
   }
}
