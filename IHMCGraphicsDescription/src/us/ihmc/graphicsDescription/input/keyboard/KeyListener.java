package us.ihmc.graphicsDescription.input.keyboard;

import us.ihmc.tools.inputDevices.keyboard.Key;

public interface KeyListener
{
   public void keyPressed(Key key);
   public void keyReleased(Key key);
}
