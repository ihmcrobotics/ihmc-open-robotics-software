package us.ihmc.avatar.stereoVision.net.packet.listener;

import us.ihmc.avatar.stereoVision.net.packet.StereoVisionImageFragmentPacket;
import us.ihmc.avatar.stereoVision.net.packet.StereoVisionPacket;
import us.ihmc.avatar.stereoVision.net.packet.StereoVisionSetupPacket;

public interface StereoVisionPacketListener
{
   default void onPacket(StereoVisionPacket packet)
   {

   }

   default void onSetupPacket(StereoVisionSetupPacket setupPacket)
   {

   }

   default void onImageFragmentPacket(StereoVisionImageFragmentPacket fragmentPacket)
   {

   }
}
