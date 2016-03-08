package us.ihmc.aware.packets;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public class BodyPosePacket extends PosePacket
{
   public BodyPosePacket()
   {
   }

   public BodyPosePacket(Point3d position, Quat4d orientation)
   {
      super(position, orientation);
   }

   public BodyPosePacket(double x, double y, double z, double yaw, double pitch, double roll)
   {
      super(x, y, z, yaw, pitch, roll);
   }
}
