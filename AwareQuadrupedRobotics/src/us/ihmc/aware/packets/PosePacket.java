package us.ihmc.aware.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.RotationTools;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public class PosePacket extends Packet<PosePacket>
{
   private Point3d position;
   private Quat4d orientation;

   public PosePacket()
   {
      this.position = new Point3d();
      this.orientation = new Quat4d();
   }

   public PosePacket(Point3d position, Quat4d orientation)
   {
      this.position = new Point3d(position);
      this.orientation = new Quat4d(orientation);
   }

   public PosePacket(double x, double y, double z, double yaw, double pitch, double roll)
   {
      this.position = new Point3d(x, y, z);
      this.orientation = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(yaw, pitch, roll, this.orientation);
   }

   public void get(Point3d position, Quat4d orientation)
   {
      position.set(this.position);
      orientation.set(this.orientation);
   }

   public void getPosition(Point3d position)
   {
      position.set(this.position);
   }

   public void getOrientation(Quat4d orientation)
   {
      orientation.set(this.orientation);
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

   public double getYaw()
   {
      return RotationTools.computeYaw(this.orientation);
   }

   public double getPitch()
   {
      return RotationTools.computePitch(this.orientation);
   }

   public double getRoll()
   {
      return RotationTools.computeRoll(this.orientation);
   }

   @Override public boolean epsilonEquals(PosePacket other, double epsilon)
   {
      return (this.position.epsilonEquals(other.position, epsilon) && this.orientation.epsilonEquals(other.orientation, epsilon));
   }
}
