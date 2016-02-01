package us.ihmc.tools.inputDevices.mouse3DJoystick;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DJoystick;
import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DListener;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class Mouse3DJoystickTest
{
	@DeployableTestMethod(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testMouse3DJoystick()
   {
      Mouse3DJoystick mouse3dJoystick = new Mouse3DJoystick();
      
      mouse3dJoystick.addMouse3DListener(new Mouse3DListener()
      {
         @Override
         public void mouseDragged(double dx, double dy, double dz, double drx, double dry, double drz)
         {
            PrintTools.info(Mouse3DJoystickTest.this, "dx: " + format(dx) + " dy: " + format(dy) + " dz: " + format(dz) + " drx: " + format(drx) + " dry: " + format(dry) + " drz: " + format(drz));
         }
      });
      
      ThreadTools.sleepSeconds(0.2);
      
      mouse3dJoystick.stopPolling();
      
      boolean userWasSatisfied = true;
      assertTrue(userWasSatisfied);
   }
   
   private String format(double value)
   {
      return FormattingTools.getFormattedDecimal2D(FormattingTools.roundToSignificantFigures(value, 2));
   }
}
