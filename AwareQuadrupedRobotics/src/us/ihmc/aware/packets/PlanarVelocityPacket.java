package us.ihmc.aware.packets;

import us.ihmc.communication.packets.Packet;

import javax.vecmath.Vector3d;

public class PlanarVelocityPacket extends Packet<PlanarVelocityPacket>
{
   private final Vector3d velocity;

   public PlanarVelocityPacket()
   {
      this.velocity = new Vector3d();
   }

   public PlanarVelocityPacket(Vector3d velocity)
   {
      this.velocity = new Vector3d(velocity);
   }

   public PlanarVelocityPacket(double xdot, double ydot, double adot)
   {
      this.velocity = new Vector3d(xdot, ydot, adot);
   }

   public void get(Vector3d velocity)
   {
      velocity.set(this.velocity);
   }

   public void getLinearVelocity(Vector3d velocity)
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

   public double getA()
   {
      return velocity.getZ();
   }

   @Override public boolean epsilonEquals(PlanarVelocityPacket other, double epsilon)
   {
      return this.velocity.epsilonEquals(other.velocity, epsilon);
   }
}
