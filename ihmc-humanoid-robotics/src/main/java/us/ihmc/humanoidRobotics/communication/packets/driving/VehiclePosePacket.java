package us.ihmc.humanoidRobotics.communication.packets.driving;

import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.RotationTools;

public class VehiclePosePacket extends Packet<VehiclePosePacket>
{
   public Point3D position = new Point3D();
   public Quaternion orientation = new Quaternion();

   public int index = 0;

   public VehiclePosePacket()
   {
      // Empty constructor for deserialization
   }

   public VehiclePosePacket(VehiclePosePacket other)
   {
      position = new Point3D(other.position);
      orientation = new Quaternion(other.orientation);
   }

   @Override
   public void set(VehiclePosePacket other)
   {
      position = new Point3D(other.position);
      orientation = new Quaternion(other.orientation);
      setPacketInformation(other);
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   @Override
   public boolean epsilonEquals(VehiclePosePacket other, double epsilon)
   {
      boolean ret = RotationTools.quaternionEpsilonEquals(getOrientation(), other.getOrientation(), epsilon);
      ret &= getPosition().epsilonEquals(other.getPosition(), epsilon);

      return ret;
   }

   @Override
   public String toString()
   {
      double[] ypr = new double[3];
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(orientation, ypr);
      String ret = "Car= (" + FormattingTools.getFormattedDecimal3D(position.getX()) + "," + FormattingTools.getFormattedDecimal3D(position.getY()) + ","
            + FormattingTools.getFormattedDecimal3D(position.getZ()) + ")," + " (" + FormattingTools.getFormattedDecimal3D(ypr[0]) + ","
            + FormattingTools.getFormattedDecimal3D(ypr[1]) + "," + FormattingTools.getFormattedDecimal3D(ypr[2]) + ")";

      return ret;
   }
}
