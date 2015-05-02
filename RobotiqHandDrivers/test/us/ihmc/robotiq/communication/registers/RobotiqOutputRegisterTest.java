package us.ihmc.robotiq.communication.registers;

import org.junit.Test;

import us.ihmc.utilities.test.AbstractUnitTest;

public abstract class RobotiqOutputRegisterTest implements AbstractUnitTest<RobotiqOutputRegister>
{
   protected abstract RobotiqRegister getRegister();
   protected abstract byte getExpectedByteValue();
   
   @Test
   public void testGetRegisterValue()
   {
      RobotiqOutputRegister outputRegister = getImplementingType();
      
      
   }
}
