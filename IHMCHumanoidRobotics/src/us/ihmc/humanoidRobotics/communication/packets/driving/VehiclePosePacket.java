package us.ihmc.humanoidRobotics.communication.packets.driving;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;

public class VehiclePosePacket extends Packet<VehiclePosePacket> implements TransformableDataObject<VehiclePosePacket>
{
   public Point3d position;
   public Quat4d orientation;

   public int index = 0;

   public VehiclePosePacket()
   {
      // Empty constructor for deserialization
   }

   public VehiclePosePacket(Point3d position, Quat4d orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }

   public VehiclePosePacket(RigidBodyTransform transformFromVehicleToWorld)
   {
      Matrix3d rotationMatrix = new Matrix3d();
      transformFromVehicleToWorld.get(rotationMatrix);
      orientation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnMatrix3d(orientation, rotationMatrix);

      Vector3d translation = new Vector3d();
      transformFromVehicleToWorld.get(translation);
      position = new Point3d(translation);
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public VehiclePosePacket transform(RigidBodyTransform transform)
   {
      VehiclePosePacket ret = new VehiclePosePacket();

      // Point3d position;
      ret.position = TransformTools.getTransformedPoint(this.getPosition(), transform);

      // Quat4d orientation;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);

      return ret;
   }

   public boolean epsilonEquals(VehiclePosePacket other, double epsilon)
   {
      boolean ret = RotationFunctions.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      ret &= this.getPosition().epsilonEquals(other.getPosition(), epsilon);

      return ret;
   }

   public String toString()
   {
      double[] ypr = new double[3];
      RotationFunctions.setYawPitchRollBasedOnQuaternion(ypr, orientation);
      String ret = ("Car= (" + FormattingTools.getFormattedDecimal3D(position.getX()) + "," + FormattingTools.getFormattedDecimal3D(position.getY()) + ","
                    + FormattingTools.getFormattedDecimal3D(position.getZ()) + ")," + " (" + FormattingTools.getFormattedDecimal3D(ypr[0]) + ","
                    + FormattingTools.getFormattedDecimal3D(ypr[1]) + "," + FormattingTools.getFormattedDecimal3D(ypr[2]) + ")");

      return ret;
   }

   public VehiclePosePacket(Random random)
   {
      Point3d point = new Point3d();
      Quat4d quat = new Quat4d();

      point.set(RandomTools.generateRandomPoint(random, 0.288, 0.288, 0.288));    // magic numbers so point will not exceed XYZ_MIN / MAX in PelvisOrientationPacketSerializer
      quat.set(RandomTools.generateRandomRotation(random));

      this.position = point;
      this.orientation = quat;
   }
}
