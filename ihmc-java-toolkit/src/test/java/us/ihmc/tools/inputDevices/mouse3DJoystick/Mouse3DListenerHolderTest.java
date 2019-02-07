package us.ihmc.tools.inputDevices.mouse3DJoystick;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListener;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListenerHolder;
public class Mouse3DListenerHolderTest
{
   int count1 = 0;
   int count2 = 0;
   
   @Test
   public void testListenersGetNotified()
   {
      Mouse3DListenerHolder holder = new Mouse3DListenerHolder();
      
      holder.addMouse3DListener(new Mouse3DListener()
      {
         @Override
         public void mouseDragged(double dx, double dy, double dz, double drx, double dry, double drz)
         {
            count1++;
         }
      });
      
      holder.addMouse3DListener(new Mouse3DListener()
      {
         @Override
         public void mouseDragged(double dx, double dy, double dz, double drx, double dry, double drz)
         {
            count2++;
         }
      });
      
      holder.mouseDragged(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
      
      assertTrue("Didn't get called.", count1 == 1 && count2 == 1);
   }
}
