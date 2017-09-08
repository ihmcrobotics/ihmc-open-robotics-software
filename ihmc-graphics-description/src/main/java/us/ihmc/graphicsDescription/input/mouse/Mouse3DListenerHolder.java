package us.ihmc.graphicsDescription.input.mouse;

import java.util.ArrayList;

public class Mouse3DListenerHolder
{
   private final ArrayList<Mouse3DListener> mouse3DListeners = new ArrayList<>();
   
   public void addMouse3DListener(Mouse3DListener mouse3DListener)
   {
      mouse3DListeners.add(mouse3DListener);
   }
   
   public void mouseDragged(double dx, double dy, double dz, double drx, double dry, double drz)
   {
      for (Mouse3DListener mouse3DListener : mouse3DListeners)
      {
         mouse3DListener.mouseDragged(dx, dy, dz, drx, dry, drz);
      }
   }
}
