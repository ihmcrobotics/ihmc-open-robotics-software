package us.ihmc.tools.inputDevices.ghostMouse;

import java.awt.event.InputEvent;

import us.ihmc.tools.inputDevices.mouse.MouseButton;


public class GhostMouseButtonReleasedPlaybackEvent implements GhostMousePlaybackEvent
{
   private final int x, y;
   private final MouseButton mouseButton;
   
   public static final int LEFT = InputEvent.BUTTON1_MASK;
   public static final int MIDDLE = InputEvent.BUTTON2_MASK;
   public static final int RIGHT = InputEvent.BUTTON3_MASK;
   
   public GhostMouseButtonReleasedPlaybackEvent(int x, int y, MouseButton mouseButton)
   {
      this.x = x;
      this.y = y;
      
      this.mouseButton = mouseButton;
   }
   
   public GhostMouseButtonReleasedPlaybackEvent(int[] coordinates, MouseButton mouseButton)
   {
      this(coordinates[0], coordinates[1], mouseButton);
   }
      
   @Override
   public void playback(java.awt.Robot awtRobot, double playbackSpeed)
   {
      awtRobot.mouseMove(x, y);
      awtRobot.mouseRelease(mouseButton.getInputEventMask());
   }
   
   @Override
   public String toString()
   {
      return "{" + mouseButton.toShortString() + "Mouse up (" + x + "," + y + ")}";
   }
   
}
