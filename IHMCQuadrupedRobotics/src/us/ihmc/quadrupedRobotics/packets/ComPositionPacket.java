package us.ihmc.quadrupedRobotics.packets;

import us.ihmc.communication.packets.Packet;

import javax.vecmath.Point3d;

public class ComPositionPacket extends Packet<ComPositionPacket>
{
   private Point3d position;

   public ComPositionPacket()
   {
      this.position = new Point3d();
   }

   public ComPositionPacket(Point3d position)
   {
      this.position = new Point3d(position);
   }

   public ComPositionPacket(double x, double y, double z)
   {
      this.position = new Point3d(x, y, z);
   }

   public void get(Point3d position)
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
