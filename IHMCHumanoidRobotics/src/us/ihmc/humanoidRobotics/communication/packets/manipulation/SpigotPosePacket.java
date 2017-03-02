package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomGeometry;

public class SpigotPosePacket extends Packet<SpigotPosePacket> implements TransformableDataObject<SpigotPosePacket>
{
   public Point3D position;
   public Quaternion orientation;

   public int index = 0;

   public SpigotPosePacket()
   {
      // Empty constructor for deserialization
   }

   public SpigotPosePacket(Point3D position, Quaternion orientation)
   {
      this.orientation = orientation;
      this.position = position;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public Point3D getPosition()
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
      boolean ret = RotationTools.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      ret &= this.getPosition().epsilonEquals(other.getPosition(), epsilon);

      return ret;
   }

   public SpigotPosePacket(Random random)
   {
      Point3D point = new Point3D();
      Quaternion quat = new Quaternion();

      point.set(RandomGeometry.nextPoint3D(random, 0.288, 0.288, 0.288));    // magic

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
      quat.set(RandomGeometry.nextAxisAngle(random));

      this.position = point;
      this.orientation = quat;
   }
}
