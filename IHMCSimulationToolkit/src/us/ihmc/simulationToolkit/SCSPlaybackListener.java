package us.ihmc.simulationToolkit;

import us.ihmc.humanoidRobotics.communication.packets.SCSListenerPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.simulationconstructionset.PlaybackListener;


public class SCSPlaybackListener implements PlaybackListener
{
   public HumanoidGlobalDataProducer networkServer;

   public SCSPlaybackListener(HumanoidGlobalDataProducer dataProducer)
   {
      this.networkServer = dataProducer;
   }

   public void play(double realTimeRate)
   {
   }

   public void stop()
   {
//      System.out.println("SCSPlaybackListener: stopped");
      if (networkServer != null)
         networkServer.queueDataToSend(new SCSListenerPacket());
   }

   public void indexChanged(int newIndex, double newTime)
   {
   }
}
