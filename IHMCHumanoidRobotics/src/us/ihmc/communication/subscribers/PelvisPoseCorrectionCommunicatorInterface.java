package us.ihmc.communication.subscribers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.sensing.LocalizationPacket;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;

public interface PelvisPoseCorrectionCommunicatorInterface extends PacketConsumer<StampedPosePacket>
{
   public boolean hasNewPose();

   public StampedPosePacket getNewExternalPose();
   
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket);
   
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket);
}
