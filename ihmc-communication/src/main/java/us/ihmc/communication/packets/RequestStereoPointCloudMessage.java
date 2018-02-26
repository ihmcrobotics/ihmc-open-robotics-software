package us.ihmc.communication.packets;

public class RequestStereoPointCloudMessage extends Packet<RequestStereoPointCloudMessage>
{

   public RequestStereoPointCloudMessage()
   {
   }

   @Override
   public void set(RequestStereoPointCloudMessage other)
   {
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(RequestStereoPointCloudMessage other, double epsilon)
   {
      return true;
   }
}
