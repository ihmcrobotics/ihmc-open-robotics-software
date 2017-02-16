package us.ihmc.tools.inputDevices.ghostMouse;

public class GhostMouseDelayPlaybackEvent implements GhostMousePlaybackEvent
{
   private final double delayInSeconds;

   public GhostMouseDelayPlaybackEvent(double delayInSeconds)
   {
      this.delayInSeconds = delayInSeconds;
   }

   @Override
   public void playback(java.awt.Robot awtRobot, double playbackSpeed)
   {
      awtRobot.delay((int) (delayInSeconds / playbackSpeed * 1000));
   }
   
   @Override
   public String toString()
   {
      return "{Delay " + delayInSeconds + "}";
   }
}
