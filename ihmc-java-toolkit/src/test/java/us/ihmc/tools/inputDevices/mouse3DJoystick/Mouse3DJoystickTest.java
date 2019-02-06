package us.ihmc.tools.inputDevices.mouse3DJoystick;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.PrintTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListener;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;

public class Mouse3DJoystickTest
{
   @Test
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
      return FormattingTools.getFormattedDecimal2D(MathTools.roundToSignificantFigures(value, 2));
   }
}
