package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class PelvisPoseCorrectionCommunicator implements PelvisPoseCorrectionCommunicatorInterface
{
   private final ConcurrentLinkedQueue<StampedPosePacket> packetQueue = new ConcurrentLinkedQueue<StampedPosePacket>();
   private HumanoidGlobalDataProducer globalDataProducer;
   
   public PelvisPoseCorrectionCommunicator(HumanoidGlobalDataProducer globalDataProducer)
   {
	   this.globalDataProducer = globalDataProducer;
   }

   public void receivedPacket(StampedPosePacket newestStampedPosePacket)
   {
      packetQueue.add(newestStampedPosePacket);
   }
   
   public boolean hasNewPose()
   {
      return !packetQueue.isEmpty();
   }
   
   @Override
   public StampedPosePacket getNewExternalPose()
   {
	   return packetQueue.poll();
   }
   
   @Override
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket) {
	   if (globalDataProducer != null)
	      globalDataProducer.queueDataToSend(pelvisPoseErrorPacket);
   }

   @Override
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
   {
      if (globalDataProducer != null)
         globalDataProducer.queueDataToSend(localizationPacket);      
   }
}
