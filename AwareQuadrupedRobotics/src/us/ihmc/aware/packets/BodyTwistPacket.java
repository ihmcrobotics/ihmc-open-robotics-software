package us.ihmc.aware.packets;

import javax.vecmath.Vector3d;

public class BodyTwistPacket extends TwistPacket
{
   public BodyTwistPacket()
   {
   }

   public BodyTwistPacket(Vector3d linearVelocity, Vector3d angularVelocity)
   {
      super(linearVelocity, angularVelocity);
   }

   public BodyTwistPacket(double vx, double vy, double vz, double wx, double wy, double wz)
   {
      super(vx, vy, vz, wx, wy, wz);
   }
}
