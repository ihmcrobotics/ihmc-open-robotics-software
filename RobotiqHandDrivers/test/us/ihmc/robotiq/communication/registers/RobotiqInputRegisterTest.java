package us.ihmc.robotiq.communication.registers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

public abstract class RobotiqInputRegisterTest
{
   protected abstract RobotiqInputRegister getInputRegister();
   protected abstract RobotiqInputRegister getExpectedRegister();
   protected abstract byte getValueToSet();
   
   @Test(timeout = 3000)
   @EstimatedDuration(duration = 0.1)
   public void testSetRegister()
   {
      RobotiqInputRegister inputRegister = getInputRegister();
      inputRegister.setRegisterValue(getValueToSet());
      
      assertTrue(getExpectedRegister().equals(inputRegister));
   }

   @Test(timeout = 3000)
   @EstimatedDuration(duration = 0.1)
   public void testGetRegister()
   {
      assertEquals(getValueToSet(), getExpectedRegister().getRegisterValue());
   }

}
