package us.ihmc.avatar.stereoVision.net.packet;

import java.nio.ByteBuffer;

public abstract class StereoVisionPacket
{
   public static final byte PACKET_ID_SETUP = 0;
   public static final byte PACKET_ID_IMAGE_FRAGMENT = 1;

   public abstract int serialize(ByteBuffer buffer);

   public abstract void deserialize(ByteBuffer buffer);

   public abstract byte getPacketID();
}
