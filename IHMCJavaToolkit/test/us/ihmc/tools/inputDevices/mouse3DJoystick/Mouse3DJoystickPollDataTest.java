package us.ihmc.tools.inputDevices.mouse3DJoystick;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DPollData;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class Mouse3DJoystickPollDataTest
{
	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPollData()
   {
      Mouse3DPollData pollData = new Mouse3DPollData(-1.0, -2.0, 0.0, 1.0, 2.0, 3.0);
      
      assertTrue("Value not set", pollData.getDx() == -1.0);
      assertTrue("Value not set", pollData.getDy() == -2.0);
      assertTrue("Value not set", pollData.getDz() == 0.0);
      assertTrue("Value not set", pollData.getDrx() == 1.0);
      assertTrue("Value not set", pollData.getDry() == 2.0);
      assertTrue("Value not set", pollData.getDrz() == 3.0);
   }
}
