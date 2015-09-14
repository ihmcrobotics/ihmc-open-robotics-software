package us.ihmc.tools.inputDevices.mouse3DJoystick;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DListener;
import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DListenerHolder;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class Mouse3DListenerHolderTest
{
   int count1 = 0;
   int count2 = 0;
   
	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
