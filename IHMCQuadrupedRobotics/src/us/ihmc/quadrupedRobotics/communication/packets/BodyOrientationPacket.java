package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple4D.Quaternion;

public class BodyOrientationPacket extends Packet<BodyOrientationPacket>
{
   private Quaternion orientation;

   public BodyOrientationPacket()
   {
      this.orientation = new Quaternion();
   }

   public BodyOrientationPacket(Quaternion orientation)
   {
      this.orientation = new Quaternion(orientation);
   }

   public BodyOrientationPacket(double yaw, double pitch, double roll)
   {
      this.orientation = new Quaternion();
      this.orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void get(Quaternion orientation)
   {
      orientation.set(this.orientation);
   }

   public double getYaw()
   {
      return this.orientation.getYaw();
   }

   public double getPitch()
   {
      return this.orientation.getPitch();
   }

   public double getRoll()
   {
      return this.orientation.getRoll();
   }

   @Override public boolean epsilonEquals(BodyOrientationPacket other, double epsilon)
   {
      return this.orientation.epsilonEquals(other.orientation, epsilon);
   }
}
