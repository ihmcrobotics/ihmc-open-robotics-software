package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

public class PointCloudReceivedPacket extends Packet<PointCloudReceivedPacket>
{
   public long latestCompleteReceivedPointCloudPacket;

   public PointCloudReceivedPacket()
   {
      setDestination(PacketDestination.TRAFFIC_SHAPER);
   }

   @Override
   public boolean epsilonEquals(PointCloudReceivedPacket other, double epsilon)
   {
      return other.latestCompleteReceivedPointCloudPacket == latestCompleteReceivedPointCloudPacket;
   }

}
