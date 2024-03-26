package us.ihmc.avatar.stereoVision.net.packet;

import java.nio.ByteBuffer;

public abstract class StereoVisionPacket
{
   public static final byte PACKET_ID_C2S_SETUP = 0;
   public static final byte PACKET_ID_S2C_FRAGMENT = 1;

   public abstract void serialize(ByteBuffer buffer);

   public abstract void deserialize(ByteBuffer buffer);
}
