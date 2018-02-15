package us.ihmc.communication.packets;

import us.ihmc.euclid.geometry.BoundingBox3D;

public class RequestPlanarRegionsListMessage extends SettablePacket<RequestPlanarRegionsListMessage>
{
   public enum RequestType {SINGLE_UPDATE, CONTINUOUS_UPDATE, STOP_UPDATE, CLEAR};

   public RequestType requestType;
   public BoundingBox3D boundingBoxInWorldForRequest;

   public RequestPlanarRegionsListMessage()
   {
   }

   public RequestPlanarRegionsListMessage(RequestType requestType)
   {
      this(requestType, null, null);
   }

   public RequestPlanarRegionsListMessage(RequestType requestType, BoundingBox3D boundingBoxInWorldForRequest)
   {
      this(requestType, boundingBoxInWorldForRequest, null);
   }

   public RequestPlanarRegionsListMessage(RequestType requestType, PacketDestination destination)
   {
      this(requestType, null, destination);
   }

   public RequestPlanarRegionsListMessage(RequestType requestType, BoundingBox3D boundingBoxInWorldForRequest, PacketDestination destination)
   {
      this.requestType = requestType;
      this.boundingBoxInWorldForRequest = boundingBoxInWorldForRequest;
      if (destination != null)
         setDestination(destination);
   }

   @Override
   public void set(RequestPlanarRegionsListMessage other)
   {
      requestType = other.requestType;
      boundingBoxInWorldForRequest = other.boundingBoxInWorldForRequest;
      setDestination(other.getDestination());
      setPacketInformation(other);
   }

   public RequestType getRequestType()
   {
      return requestType;
   }

   public void setRequestType(RequestType requestType)
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
