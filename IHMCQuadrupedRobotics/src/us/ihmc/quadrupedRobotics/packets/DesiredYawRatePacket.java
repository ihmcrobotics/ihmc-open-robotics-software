package us.ihmc.quadrupedRobotics.packets;

import us.ihmc.communication.packets.Packet;

public class DesiredYawRatePacket extends Packet<DesiredYawRatePacket>
{
   private double yawRate;

   public DesiredYawRatePacket(){}

   public DesiredYawRatePacket(double yawRate)
   {
      this.yawRate = yawRate;
   }

   public double getYawRate()
   {
      return yawRate;
   }

   @Override public boolean epsilonEquals(DesiredYawRatePacket other, double epsilon)
   {
      return (this.yawRate - other.getYawRate()) <= epsilon;
   }
}
