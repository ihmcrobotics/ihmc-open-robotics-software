package us.ihmc.communication.packets;

import us.ihmc.euclid.geometry.BoundingBox3D;

public class RequestPlanarRegionsListMessage extends SettablePacket<RequestPlanarRegionsListMessage>
{
   public static final byte SINGLE_UPDATE = 0;
   public static final byte CONTINUOUS_UPDATE = 1;
   public static final byte STOP_UPDATE = 2;
   public static final byte CLEAR = 3;

   public byte planarRegionsRequestType;
   public BoundingBox3D boundingBoxInWorldForRequest;

   public RequestPlanarRegionsListMessage()
   {
   }

   @Override
   public void set(RequestPlanarRegionsListMessage other)
   {
      planarRegionsRequestType = other.planarRegionsRequestType;
      boundingBoxInWorldForRequest = other.boundingBoxInWorldForRequest;
      setDestination(other.getDestination());
      setPacketInformation(other);
   }

   public byte getRequestType()
   {
      return planarRegionsRequestType;
   }

   public void setRequestType(byte requestType)
   {
      this.planarRegionsRequestType = requestType;
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
      return planarRegionsRequestType == other.planarRegionsRequestType;
   }
}
