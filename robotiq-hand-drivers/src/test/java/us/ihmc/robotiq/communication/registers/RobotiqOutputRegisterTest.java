package us.ihmc.robotiq.communication.registers;

import static us.ihmc.robotics.Assert.*;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public abstract class RobotiqOutputRegisterTest
{
   protected abstract byte getExpectedByteValue();
   protected abstract RobotiqOutputRegister getOutputRegister();
   
   @Test(timeout = 30000)
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testGetRegisterValue()
   {
      byte expectedValue = getExpectedByteValue();
      RobotiqOutputRegister outputRegister = getOutputRegister();
      
      assertEquals(expectedValue, outputRegister.getRegisterValue());
   }
}
