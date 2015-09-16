package us.ihmc.humanoidRobotics.communication.subscribers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;

public interface PelvisPoseCorrectionCommunicatorInterface extends PacketConsumer<StampedPosePacket>
{
   public boolean hasNewPose();

   public StampedPosePacket getNewExternalPose();
   
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket);
   
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket);
}
