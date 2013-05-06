package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.utilities.net.ObjectCommunicator;

import com.yobotics.simulationconstructionset.PlaybackListener;


public class SCSPlaybackListener implements PlaybackListener
{
   public ObjectCommunicator networkServer;

   public SCSPlaybackListener(ObjectCommunicator networkServer)
   {
      this.networkServer = networkServer;
   }

   public void play(double realTimeRate)
   {
   }

   public void stop()
   {
//      System.out.println("SCSPlaybackListener: stopped");
      networkServer.consumeObject(new SCSListenerPacket());
   }

   public void indexChanged(int newIndex, double newTime)
   {
   }
}
