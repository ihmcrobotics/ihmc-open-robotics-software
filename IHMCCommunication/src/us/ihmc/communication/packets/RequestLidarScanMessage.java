package us.ihmc.communication.packets;

public class RequestLidarScanMessage extends TrackablePacket<RequestLidarScanMessage>
{
   public RequestLidarScanMessage()
   {
   }

   @Override
   public boolean epsilonEquals(RequestLidarScanMessage other, double epsilon)
   {
      return true;
   }
}
