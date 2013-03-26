package us.ihmc.darpaRoboticsChallenge;

import com.yobotics.simulationconstructionset.PlaybackListener;
import us.ihmc.utilities.net.KryoObjectServer;

/**
 * User: Matt
 * Date: 3/26/13
 */
public class SCSPlaybackListener implements PlaybackListener
{
   public KryoObjectServer networkServer;

   public SCSPlaybackListener(KryoObjectServer networkServer)
   {
      this.networkServer = networkServer;
   }

   public void play(double realTimeRate)
   {
   }

   public void stop()
   {
      System.out.println("stopped");
      networkServer.consumeObject(new SCSListenerPacket());
   }

   public void indexChanged(int newIndex, double newTime)
   {
   }
}
