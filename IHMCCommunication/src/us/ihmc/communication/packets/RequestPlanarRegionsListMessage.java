package us.ihmc.communication.packets;

public class RequestPlanarRegionsListMessage extends TrackablePacket<RequestPlanarRegionsListMessage>
{
   public enum RequestType {SINGLE_UPDATE, CONTINUOUS_UPDATE, STOP_UPDATE};

   public RequestType requestType;

   public RequestPlanarRegionsListMessage()
   {
   }
   
   public RequestPlanarRegionsListMessage(RequestType requestType)
   {
      this.requestType = requestType;
   }

   public RequestType getRequesType()
   {
      return requestType;
   }

   public void setRequesType(RequestType requesType)
   {
      this.requestType = requesType;
   }

   @Override
   public boolean epsilonEquals(RequestPlanarRegionsListMessage other, double epsilon)
   {
      return requestType == other.requestType;
   }
}
