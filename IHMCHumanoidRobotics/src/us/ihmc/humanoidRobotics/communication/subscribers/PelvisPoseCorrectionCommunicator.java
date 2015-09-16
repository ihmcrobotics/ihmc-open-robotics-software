package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.GlobalDataProducer;

public class PelvisPoseCorrectionCommunicator implements PelvisPoseCorrectionCommunicatorInterface
{
   private final ConcurrentLinkedQueue<StampedPosePacket> packetQueue = new ConcurrentLinkedQueue<StampedPosePacket>();
   private GlobalDataProducer globalDataProducer;
   
   public PelvisPoseCorrectionCommunicator(GlobalDataProducer globalDataProducer)
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
