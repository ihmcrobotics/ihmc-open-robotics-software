package us.ihmc.robotiq.communication.registers;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public abstract class RobotiqOutputRegisterTest
{
   protected abstract byte getExpectedByteValue();
   protected abstract RobotiqOutputRegister getOutputRegister();
   
   @Test
   public void testGetRegisterValue()
   {
      byte expectedValue = getExpectedByteValue();
      RobotiqOutputRegister outputRegister = getOutputRegister();
      
      assertEquals(expectedValue, outputRegister.getRegisterValue());
   }
}
