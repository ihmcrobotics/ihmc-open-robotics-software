package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

import us.ihmc.euclid.tuple3D.Vector3D;

public class BodyAngularRatePacket extends Packet<BodyAngularRatePacket>
{
   private final Vector3D velocity;

   public BodyAngularRatePacket()
   {
      this.velocity = new Vector3D();
   }

   public BodyAngularRatePacket(Vector3D velocity)
   {
      this.velocity = new Vector3D(velocity);
   }

   public BodyAngularRatePacket(double wx, double wy, double wz)
   {
      this.velocity = new Vector3D(wx, wy, wz);
   }

   public void get(Vector3D velocity)
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
