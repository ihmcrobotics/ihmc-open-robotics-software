package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gACT;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gGTO;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gIMC;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gMOD;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gSTA;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;

@DeployableTestClass(planType = BambooPlanType.Fast)
public class GripperStatusRegisterTest extends RobotiqInputRegisterTest
{
   @Override
   protected RobotiqInputRegister getInputRegister()
   {
      return new GripperStatusRegister(gACT.GRIPPER_RESET, gMOD.BASIC_MODE, gGTO.STOPPED, gIMC.GRIPPER_IN_RESET, gSTA.GRIPPER_IN_MOTION);
   }

   @Override
   protected RobotiqInputRegister getExpectedRegister()
   {
      return new GripperStatusRegister(gACT.GRIPPER_ACTIVATION, gMOD.SCISSOR_MODE, gGTO.GO_TO, gIMC.ACTIVATION_IN_PROGRESS, gSTA.ONE_OR_TWO_FINGERS_STOPPED);
   }

   @Override
   protected byte getValueToSet()
   {
      return 95;
   }

}
