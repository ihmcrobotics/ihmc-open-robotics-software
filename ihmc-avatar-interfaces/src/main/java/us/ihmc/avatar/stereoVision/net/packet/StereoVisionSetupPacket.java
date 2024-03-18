package us.ihmc.avatar.stereoVision.net.packet;

import java.nio.ByteBuffer;

// TODO:
public class StereoVisionSetupPacket extends StereoVisionPacket
{
   @Override
   public int serialize(ByteBuffer buffer)
   {
      return 0;
   }

   @Override
   public void deserialize(ByteBuffer buffer)
   {

   }

   @Override
   public byte getPacketID()
   {
      return StereoVisionPacket.PACKET_ID_SETUP;
   }
}
