package us.ihmc.avatar.stereoVision.net.packet;

public interface StereoVisionPacketListener
{
   default void onPacket(StereoVisionPacket packet)
   {

   }

   default void onSetupPacket(StereoVisionSetupPacket setupPacket)
   {

   }

   default void onImageFragmentPacket(StereoVisionImageFragmentPacket imageFragmentPacket)
   {

   }
}
