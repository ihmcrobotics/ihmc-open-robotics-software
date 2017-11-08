package us.ihmc.communication.packets;

public class RequestPlanarRegionsListMessage extends RequestPacket<RequestPlanarRegionsListMessage>
{
   public enum RequestType {SINGLE_UPDATE, CONTINUOUS_UPDATE, STOP_UPDATE, CLEAR};

   public RequestType requestType;

   public RequestPlanarRegionsListMessage()
   {
   }

   public RequestPlanarRegionsListMessage(RequestType requestType)
   {
      this.requestType = requestType;
   }

   public RequestPlanarRegionsListMessage(RequestType requestType, PacketDestination destination)
   {
      this.requestType = requestType;
      setDestination(destination);
   }

   public RequestType getRequestType()
   {
      return requestType;
   }

   public void setRequestType(RequestType requestType)
   {
      this.requestType = requestType;
   }

   @Override
   public boolean epsilonEquals(RequestPlanarRegionsListMessage other, double epsilon)
   {
      return requestType == other.requestType;
   }

   public void set(RequestPlanarRegionsListMessage other)
   {
      this.requestType = other.requestType;
   }
}
