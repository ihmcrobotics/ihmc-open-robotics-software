package us.ihmc.valkyrie.generatedTestSuites;

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
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndPelvisTrajectoryMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToWholeBodyTrajectoryMessageTest.class,
   us.ihmc.valkyrie.kinematics.transmissions.ComparePushRodTransmissionsTest.class,
   us.ihmc.valkyrie.kinematics.transmissions.InefficientPushrodTransmissionJacobianTest.class,
   us.ihmc.valkyrie.kinematics.transmissions.InefficientPushRodTransmissionTest.class,
   us.ihmc.valkyrie.kinematics.util.ClosedFormJacobianTest.class
})

public class ValkyrieHFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
