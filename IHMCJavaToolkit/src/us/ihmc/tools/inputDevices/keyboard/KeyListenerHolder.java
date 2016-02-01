package us.ihmc.tools.inputDevices.keyboard;

import java.util.ArrayList;

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
