package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.humanoidRobotics.communication.packets.SCSListenerPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.GlobalDataProducer;
import us.ihmc.simulationconstructionset.PlaybackListener;


public class SCSPlaybackListener implements PlaybackListener
{
   public GlobalDataProducer networkServer;

   public SCSPlaybackListener(GlobalDataProducer dataProducer)
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
