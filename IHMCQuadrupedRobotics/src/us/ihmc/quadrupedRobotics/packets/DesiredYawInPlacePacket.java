package us.ihmc.quadrupedRobotics.packets;

import us.ihmc.communication.packets.Packet;

public class DesiredYawInPlacePacket extends Packet<DesiredYawInPlacePacket>
{
   private double yawOffset;

   public DesiredYawInPlacePacket(){}

   public DesiredYawInPlacePacket(double yawOffset)
   {
      this.yawOffset = yawOffset;
   }

   public double getYawOffset()
   {
      return yawOffset;
   }

   @Override public boolean epsilonEquals(DesiredYawInPlacePacket other, double epsilon)
   {
      return (this.yawOffset - other.getYawOffset()) <= epsilon;
   }
}
