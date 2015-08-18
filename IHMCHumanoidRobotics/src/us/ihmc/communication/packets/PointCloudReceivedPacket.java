package us.ihmc.communication.packets;

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
