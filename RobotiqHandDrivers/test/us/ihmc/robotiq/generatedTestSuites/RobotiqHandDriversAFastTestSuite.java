package us.ihmc.robotiq.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses
({
   us.ihmc.robotiq.communication.registers.ActionRequestRegisterTest.class,
   us.ihmc.robotiq.communication.registers.GripperOptionRegisterTest.class,
   us.ihmc.robotiq.communication.registers.GripperStatusRegisterTest.class,
   us.ihmc.robotiq.communication.registers.ObjectDetectionRegisterTest.class,
   us.ihmc.robotiq.communication.registers.RobotiqRegisterTest.class,
   us.ihmc.robotiq.communication.RobotiqReadResponseTest.class,
   us.ihmc.robotiq.communication.RobotiqWriteRequestFactoryTest.class
})

public class RobotiqHandDriversAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
