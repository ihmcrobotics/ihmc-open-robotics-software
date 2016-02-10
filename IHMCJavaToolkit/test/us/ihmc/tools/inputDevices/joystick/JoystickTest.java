package us.ihmc.tools.inputDevices.joystick;

import org.junit.Test;

import net.java.games.input.Event;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class JoystickTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCreateJoystick()
   {
      try
      {
         Joystick joystick = new Joystick();
         joystick.addJoystickEventListener(new JoystickEventListener()
         {
            @Override
            public void processEvent(Event event)
            {
               System.out.println(event);
            }
         });
         joystick.addJoystickStatusListener(new JoystickStatusListener()
         {
            @Override
            public void updateConnectivity(boolean connected)
            {
               System.out.println(connected);
            }
         });
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
