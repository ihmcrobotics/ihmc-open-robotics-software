package us.ihmc.tools.inputDevices.ghostMouse;

public class GhostMouseKeyReleasePlaybackEvent implements GhostMousePlaybackEvent
{
   private final int keyCode;

   public GhostMouseKeyReleasePlaybackEvent(int keyCode)
   {
      this.keyCode = keyCode;
   }

   public GhostMouseKeyReleasePlaybackEvent(String keyReleaseString)
   {
      this(GhostMouseStringToChar.convertStringToKeycode(keyReleaseString));
   }
   
   public void playback(java.awt.Robot awtRobot, double playbackSpeed)
   {
      awtRobot.keyRelease(keyCode);
   }

   public String toString()
   {
      String keyPressString = GhostMouseStringToChar.convertKeycodeToString(keyCode);
      
      return "{" + keyPressString + " up}";
   }
   
}
