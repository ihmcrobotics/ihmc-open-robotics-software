package us.ihmc.simulationToolkit;

import controller_msgs.msg.dds.SCSListenerPacket;
import us.ihmc.communication.packets.PacketDestination;
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
      {
         SCSListenerPacket packet = new SCSListenerPacket();
         packet.setDestination(PacketDestination.UI.ordinal());
         networkServer.queueDataToSend(packet);
      }
   }

   public void indexChanged(int newIndex)
   {
   }
}
