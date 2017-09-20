package us.ihmc.graphicsDescription.input.mouse;

import java.util.ArrayList;

public class MouseListenerHolder
{
   private final ArrayList<MouseListener> mouseListeners = new ArrayList<MouseListener>();

   public void addMouseListener(MouseListener mouseListener)
   {
      mouseListeners.add(mouseListener);
   }

   public void mouseDragged(MouseButton mouseButton, double dx, double dy)
   {
      for (MouseListener mouseListener : mouseListeners)
      {
         mouseListener.mouseDragged(mouseButton, dx, dy);
      }
   }
}
