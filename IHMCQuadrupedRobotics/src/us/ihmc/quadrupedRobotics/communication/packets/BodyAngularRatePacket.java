package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

import javax.vecmath.Vector3d;

public class BodyAngularRatePacket extends Packet<BodyAngularRatePacket>
{
   private final Vector3d velocity;

   public BodyAngularRatePacket()
   {
      this.velocity = new Vector3d();
   }

   public BodyAngularRatePacket(Vector3d velocity)
   {
      this.velocity = new Vector3d(velocity);
   }

   public BodyAngularRatePacket(double wx, double wy, double wz)
   {
      this.velocity = new Vector3d(wx, wy, wz);
   }

   public void get(Vector3d velocity)
   {
      velocity.set(this.velocity);
   }

   public double getX()
   {
      return velocity.getX();
   }

   public double getY()
   {
      return velocity.getY();
   }

   public double getZ()
   {
      return velocity.getZ();
   }

   @Override public boolean epsilonEquals(BodyAngularRatePacket other, double epsilon)
   {
      return this.velocity.epsilonEquals(other.velocity, epsilon);
   }
}
