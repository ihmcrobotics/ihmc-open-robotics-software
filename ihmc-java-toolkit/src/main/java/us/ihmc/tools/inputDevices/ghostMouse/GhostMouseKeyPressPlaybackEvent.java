package us.ihmc.tools.inputDevices.ghostMouse;

public class GhostMouseKeyPressPlaybackEvent implements GhostMousePlaybackEvent
{
   private final int keyCode;

   public GhostMouseKeyPressPlaybackEvent(int keyCode)
   {
      this.keyCode = keyCode;
   }
   
   public GhostMouseKeyPressPlaybackEvent(String keyPressString)
   {
      this(GhostMouseStringToChar.convertStringToKeycode(keyPressString));
   }

   @Override
   public void playback(java.awt.Robot awtRobot, double playbackSpeed)
   {
      awtRobot.keyPress(keyCode);
   }
   
   @Override
   public String toString()
   {
      String keyPressString = GhostMouseStringToChar.convertKeycodeToString(keyCode);
      
      return "{" + keyPressString + " down}";
   }

}
