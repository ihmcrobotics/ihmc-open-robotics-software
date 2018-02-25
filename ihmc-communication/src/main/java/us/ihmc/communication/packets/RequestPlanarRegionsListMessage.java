package us.ihmc.communication.packets;

public class RequestPlanarRegionsListMessage extends Packet<RequestPlanarRegionsListMessage>
{
   public static final byte SINGLE_UPDATE = 0;
   public static final byte CONTINUOUS_UPDATE = 1;
   public static final byte STOP_UPDATE = 2;
   public static final byte CLEAR = 3;

   public byte planarRegionsRequestType;
   public BoundingBox3DMessage boundingBoxInWorldForRequest = new BoundingBox3DMessage();

   public RequestPlanarRegionsListMessage()
   {
   }

   @Override
   public void set(RequestPlanarRegionsListMessage other)
   {
      planarRegionsRequestType = other.planarRegionsRequestType;
      boundingBoxInWorldForRequest.set(other.boundingBoxInWorldForRequest);
      setDestination(other.getDestination());
      setPacketInformation(other);
   }

   public byte getPlanarRegionsRequestType()
   {
      return planarRegionsRequestType;
   }

   public void setPlanarRegionsRequestType(byte requestType)
   {
      this.planarRegionsRequestType = requestType;
   }

   public BoundingBox3DMessage getBoundingBoxInWorldForRequest()
   {
      return boundingBoxInWorldForRequest;
   }

   @Override
   public boolean epsilonEquals(RequestPlanarRegionsListMessage other, double epsilon)
   {
      return planarRegionsRequestType == other.planarRegionsRequestType;
   }
}
