package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
// TODO: PacketConsumer doesn't exist in jros2 - this interface needs refactoring
// import us.ihmc.communication.net.PacketConsumer;

public interface PelvisPoseCorrectionCommunicatorInterface // extends PacketConsumer<StampedPosePacket>
{
   public boolean hasNewPose();

   public StampedPosePacket getNewExternalPose();
   
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket);
   
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket);

   public void receivedPacket(StampedPosePacket packet);
}
