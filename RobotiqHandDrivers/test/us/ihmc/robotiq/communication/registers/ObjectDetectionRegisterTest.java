package us.ihmc.robotiq.communication.registers;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTA;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTB;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTC;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTS;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
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
