package us.ihmc.graphicsDescription.input.keyboard;

import java.util.ArrayList;

import us.ihmc.tools.inputDevices.keyboard.Key;

public class KeyListenerHolder
{
   private final ArrayList<KeyListener> keyListeners = new ArrayList<KeyListener>();

   public void addKeyListener(KeyListener listener)
   {
      keyListeners.add(listener);
   }

   public void keyPressed(Key key)
   {
      for (KeyListener keyListener : keyListeners)
      {
         keyListener.keyPressed(key);
      }
   }

   public void keyReleased(Key key)
   {
      for (KeyListener keyListener : keyListeners)
      {
         keyListener.keyReleased(key);
      }
   }
}
