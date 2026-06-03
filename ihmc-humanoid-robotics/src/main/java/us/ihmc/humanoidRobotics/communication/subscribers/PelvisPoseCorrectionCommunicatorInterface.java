package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
public interface PelvisPoseCorrectionCommunicatorInterface
{
   public boolean hasNewPose();

   public StampedPosePacket getNewExternalPose();
   
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket);
   
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket);

   public void receivedPacket(StampedPosePacket packet);
}
