package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

import us.ihmc.euclid.tuple3D.Point3D;

public class ComPositionPacket extends Packet<ComPositionPacket>
{
   private Point3D position;

   public ComPositionPacket()
   {
      this.position = new Point3D();
   }

   public ComPositionPacket(Point3D position)
   {
      this.position = new Point3D(position);
   }

   public ComPositionPacket(double x, double y, double z)
   {
      this.position = new Point3D(x, y, z);
   }

   public void get(Point3D position)
   {
      position.set(this.position);
   }

   public double getX()
   {
      return this.position.getX();
   }

   public double getY()
   {
      return this.position.getY();
   }

   public double getZ()
   {
      return this.position.getZ();
   }

   @Override public boolean epsilonEquals(ComPositionPacket other, double epsilon)
   {
      return this.position.epsilonEquals(other.position, epsilon);
   }
}
