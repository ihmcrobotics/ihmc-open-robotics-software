package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

import us.ihmc.euclid.tuple3D.Vector3D;

public class ComVelocityPacket extends Packet<ComVelocityPacket>
{
   public Vector3D velocity;

   public ComVelocityPacket()
   {
      this.velocity = new Vector3D();
   }

   public ComVelocityPacket(Vector3D velocity)
   {
      this.velocity = new Vector3D(velocity);
   }

   public ComVelocityPacket(double vx, double vy, double vz)
   {
      this.velocity = new Vector3D(vx, vy, vz);
   }

   @Override
   public void set(ComVelocityPacket other)
   {
      velocity = new Vector3D(other.velocity);
      setPacketInformation(other);
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

   @Override public boolean epsilonEquals(ComVelocityPacket other, double epsilon)
   {
      return this.velocity.epsilonEquals(other.velocity, epsilon);
   }
}
