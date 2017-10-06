package us.ihmc.robotiq.communication.registers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public abstract class RobotiqInputRegisterTest
{
   protected abstract RobotiqInputRegister getInputRegister();
   protected abstract RobotiqInputRegister getExpectedRegister();
   protected abstract byte getValueToSet();
   
   @Test(timeout = 30000)
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testSetRegister()
   {
      RobotiqInputRegister inputRegister = getInputRegister();
      inputRegister.setRegisterValue(getValueToSet());
      
      assertTrue(getExpectedRegister().equals(inputRegister));
   }

   @Test(timeout = 30000)
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testGetRegister()
   {
      assertEquals(getValueToSet(), getExpectedRegister().getRegisterValue());
   }

}
