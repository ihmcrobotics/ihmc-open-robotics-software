package us.ihmc.communication.packets;

import us.ihmc.communication.ros.generators.RosIgnoredField;

public abstract class TrackablePacket<T extends TrackablePacket<T>> extends Packet<T>
{
   @RosIgnoredField
   public byte source = (byte) PacketDestination.BROADCAST.ordinal();

   public void setSource(int source)
   {
      this.source = (byte) source;
   }

   public int getSource()
   {
      return source;
   }
}
