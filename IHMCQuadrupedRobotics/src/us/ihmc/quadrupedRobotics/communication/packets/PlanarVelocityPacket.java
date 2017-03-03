package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

import us.ihmc.euclid.tuple3D.Vector3D;

public class PlanarVelocityPacket extends Packet<PlanarVelocityPacket>
{
   private final Vector3D velocity;

   public PlanarVelocityPacket()
   {
      this.velocity = new Vector3D();
   }

   public PlanarVelocityPacket(Vector3D velocity)
   {
      this.velocity = new Vector3D(velocity);
   }

   public PlanarVelocityPacket(double vx, double vy, double wz)
   {
      this.velocity = new Vector3D(vx, vy, wz);
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

   @Override public boolean epsilonEquals(PlanarVelocityPacket other, double epsilon)
   {
      return this.velocity.epsilonEquals(other.velocity, epsilon);
   }
}
