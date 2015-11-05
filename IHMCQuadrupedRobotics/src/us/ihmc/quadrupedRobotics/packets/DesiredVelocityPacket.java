package us.ihmc.quadrupedRobotics.packets;

import us.ihmc.communication.packets.Packet;

import javax.vecmath.Vector3d;

public class DesiredVelocityPacket extends Packet<DesiredVelocityPacket>
{
   private Vector3d velocity;

   public DesiredVelocityPacket(){}

   public DesiredVelocityPacket(Vector3d velocity)
   {
      this.velocity = velocity;
   }

   public DesiredVelocityPacket(double x, double y, double z)
   {
      this.velocity = new Vector3d(x, y, z);
   }

   public Vector3d getVelocity()
   {
      return velocity;
   }

   @Override public boolean epsilonEquals(DesiredVelocityPacket other, double epsilon)
   {
      return this.velocity.epsilonEquals(other.getVelocity(), epsilon);
   }
}
