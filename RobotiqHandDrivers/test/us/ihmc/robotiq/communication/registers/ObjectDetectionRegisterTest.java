package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTA;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTB;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTC;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTS;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;

@DeployableTestClass(planType = BambooPlanType.Fast)
public class ObjectDetectionRegisterTest extends RobotiqInputRegisterTest
{

   @Override
   protected RobotiqInputRegister getInputRegister()
   {
      return new ObjectDetectionRegister(gDTA.FINGER_A_AT_REQUESTED_POSITION, gDTB.FINGER_B_AT_REQUESTED_POSITION, gDTC.FINGER_C_AT_REQUESTED_POSITION, gDTS.SCISSOR_AT_REQUESTED_POSITION);
   }

   @Override
   protected RobotiqInputRegister getExpectedRegister()
   {
      return new ObjectDetectionRegister(gDTA.FINGER_A_CONTACT_WHILE_CLOSING, gDTB.FINGER_B_IN_MOTION, gDTC.FINGER_C_AT_REQUESTED_POSITION, gDTS.SCISSOR_CONTACT_WHILE_OPENING);
   }

   @Override
   protected byte getValueToSet()
   {
      return 114;
   }

}
