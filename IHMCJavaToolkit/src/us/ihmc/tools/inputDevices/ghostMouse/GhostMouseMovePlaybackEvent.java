package us.ihmc.tools.inputDevices.ghostMouse;

public class GhostMouseMovePlaybackEvent implements GhostMousePlaybackEvent
{
   private final int x, y;
   
   public GhostMouseMovePlaybackEvent(int x, int y)
   {
      this.x = x;
      this.y = y;
   }
   
   public GhostMouseMovePlaybackEvent(int[] coordinates)
   {
      this(coordinates[0], coordinates[1]);
   }
   
   @Override
   public void playback(java.awt.Robot awtRobot, double playbackSpeed)
   {
      awtRobot.mouseMove(x, y);
   }
   
   @Override
   public String toString()
   {
      return "{Move (" + x + "," + y + ")}";
   }
}
