package us.ihmc.tools.inputDevices.mouse3DJoystick;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DPollData;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class Mouse3DJoystickPollDataTest
{
   @Test
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
