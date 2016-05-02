package us.ihmc.quadrupedRobotics.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.RotationTools;

import javax.vecmath.Quat4d;

public class BodyOrientationPacket extends Packet<BodyOrientationPacket>
{
   private Quat4d orientation;

   public BodyOrientationPacket()
   {
      this.orientation = new Quat4d();
   }

   public BodyOrientationPacket(Quat4d orientation)
   {
      this.orientation = new Quat4d(orientation);
   }

   public BodyOrientationPacket(double yaw, double pitch, double roll)
   {
      this.orientation = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(yaw, pitch, roll, this.orientation);
   }

   public void get(Quat4d orientation)
   {
      orientation.set(this.orientation);
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

   @Override public boolean epsilonEquals(BodyOrientationPacket other, double epsilon)
   {
      return this.orientation.epsilonEquals(other.orientation, epsilon);
   }
}
