package us.ihmc.robotiq.communication.registers;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;

public abstract class RobotiqOutputRegisterTest
{
   protected abstract byte getExpectedByteValue();
   protected abstract RobotiqOutputRegister getOutputRegister();
   
   @Test(timeout = 3000)
   @EstimatedDuration(duration = 0.1)
   public void testGetRegisterValue()
   {
      byte expectedValue = getExpectedByteValue();
      RobotiqOutputRegister outputRegister = getOutputRegister();
      
      assertEquals(expectedValue, outputRegister.getRegisterValue());
   }
}
