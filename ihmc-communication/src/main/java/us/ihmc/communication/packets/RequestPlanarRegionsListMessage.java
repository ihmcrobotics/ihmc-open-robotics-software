package us.ihmc.communication.packets;

import us.ihmc.euclid.geometry.BoundingBox3D;

public class RequestPlanarRegionsListMessage extends SettablePacket<RequestPlanarRegionsListMessage>
{
   public byte requestType;
   public BoundingBox3D boundingBoxInWorldForRequest;

   public RequestPlanarRegionsListMessage()
   {
   }

   @Override
   public void set(RequestPlanarRegionsListMessage other)
   {
      requestType = other.requestType;
      boundingBoxInWorldForRequest = other.boundingBoxInWorldForRequest;
      setDestination(other.getDestination());
      setPacketInformation(other);
   }

   public byte getRequestType()
   {
      return requestType;
   }

   public void setRequestType(byte requestType)
   {
      this.requestType = requestType;
   }

   public boolean hasBoundingBox()
   {
      return boundingBoxInWorldForRequest != null;
   }

   public BoundingBox3D getBoundingBoxInWorldForRequest()
   {
      return boundingBoxInWorldForRequest;
   }

   @Override
   public boolean epsilonEquals(RequestPlanarRegionsListMessage other, double epsilon)
   {
      return requestType == other.requestType;
   }
}
