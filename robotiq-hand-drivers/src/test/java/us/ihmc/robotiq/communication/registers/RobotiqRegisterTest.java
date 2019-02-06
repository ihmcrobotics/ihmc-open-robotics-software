package us.ihmc.robotiq.communication.registers;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rACT;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rATR;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rGTO;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rMOD;

public class RobotiqRegisterTest
{
   @Test
   public void testEquals()
   {
      ActionRequestRegister arr = new ActionRequestRegister(rACT.ACTIVATE_GRIPPER, rMOD.BASIC_MODE, rGTO.GO_TO, rATR.NORMAL);
      ActionRequestRegister arrEqual= new ActionRequestRegister(rACT.ACTIVATE_GRIPPER, rMOD.BASIC_MODE, rGTO.GO_TO, rATR.NORMAL);
      ActionRequestRegister arrUnequal= new ActionRequestRegister(rACT.ACTIVATE_GRIPPER, rMOD.PINCH_MODE, rGTO.GO_TO, rATR.NORMAL);
      
      assertTrue(arr.equals(arrEqual));
      assertFalse(arr.equals(arrUnequal));
      
      FingerSpeedRegister fsr = new FingerSpeedRegister(Finger.FINGER_A);
      fsr.setSpeed(arr.getRegisterValue());
      assertFalse(arr.equals(fsr));
   }

}
