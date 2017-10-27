package us.ihmc.communication.packets;

public class RequestStereoPointCloudMessage extends TrackablePacket<RequestStereoPointCloudMessage>
{

   public RequestStereoPointCloudMessage()
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public boolean epsilonEquals(RequestStereoPointCloudMessage other, double epsilon)
   {
      return true;
   }
}
