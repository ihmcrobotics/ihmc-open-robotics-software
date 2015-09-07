package us.ihmc.robotiq.communication.registers;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

public abstract class RobotiqOutputRegisterTest
{
   protected abstract byte getExpectedByteValue();
   protected abstract RobotiqOutputRegister getOutputRegister();
   
   @Test(timeout = 30000)
	@EstimatedDuration(duration = 0.0)
   public void testGetRegisterValue()
   {
      byte expectedValue = getExpectedByteValue();
      RobotiqOutputRegister outputRegister = getOutputRegister();
      
      assertEquals(expectedValue, outputRegister.getRegisterValue());
   }
}
