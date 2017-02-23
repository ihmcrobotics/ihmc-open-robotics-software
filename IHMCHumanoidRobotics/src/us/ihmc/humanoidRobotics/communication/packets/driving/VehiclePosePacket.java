package us.ihmc.humanoidRobotics.communication.packets.driving;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.FormattingTools;

public class VehiclePosePacket extends Packet<VehiclePosePacket> implements TransformableDataObject<VehiclePosePacket>
{
   public Point3D position;
   public Quaternion orientation;

   public int index = 0;

   public VehiclePosePacket()
   {
      // Empty constructor for deserialization
   }

   public VehiclePosePacket(Point3D position, Quaternion orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }

   public VehiclePosePacket(RigidBodyTransform transformFromVehicleToWorld)
   {
      RotationMatrix rotationMatrix = new RotationMatrix();
      transformFromVehicleToWorld.getRotation(rotationMatrix);
      orientation = new Quaternion();
      orientation.set(rotationMatrix);

      Vector3D translation = new Vector3D();
      transformFromVehicleToWorld.getTranslation(translation);
      position = new Point3D(translation);
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public VehiclePosePacket transform(RigidBodyTransform transform)
   {
      VehiclePosePacket ret = new VehiclePosePacket();

      // Point3D position;
      ret.position = TransformTools.getTransformedPoint(this.getPosition(), transform);

      // Quat4d orientation;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);

      return ret;
   }

   public boolean epsilonEquals(VehiclePosePacket other, double epsilon)
   {
      boolean ret = RotationTools.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      ret &= this.getPosition().epsilonEquals(other.getPosition(), epsilon);

      return ret;
   }

   public String toString()
   {
      double[] ypr = new double[3];
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(orientation, ypr);
      String ret = ("Car= (" + FormattingTools.getFormattedDecimal3D(position.getX()) + "," + FormattingTools.getFormattedDecimal3D(position.getY()) + ","
                    + FormattingTools.getFormattedDecimal3D(position.getZ()) + ")," + " (" + FormattingTools.getFormattedDecimal3D(ypr[0]) + ","
                    + FormattingTools.getFormattedDecimal3D(ypr[1]) + "," + FormattingTools.getFormattedDecimal3D(ypr[2]) + ")");

      return ret;
   }

   public VehiclePosePacket(Random random)
   {
      Point3D point = new Point3D();
      Quaternion quat = new Quaternion();

      point.set(RandomTools.generateRandomPoint(random, 0.288, 0.288, 0.288));    // magic numbers so point will not exceed XYZ_MIN / MAX in PelvisOrientationPacketSerializer
      quat.set(RandomTools.generateRandomRotation(random));

      this.position = point;
      this.orientation = quat;
   }
}
