package us.ihmc.valkyrie.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.IN_DEVELOPMENT)
@SuiteClasses
({
   us.ihmc.valkyrie.kinematics.transmissions.ComparePushRodTransmissionsTest.class,
   us.ihmc.valkyrie.kinematics.util.ClosedFormJacobianTest.class,
   us.ihmc.valkyrie.obstacleCourse.ValkyrieObstacleCourseFlatTest.class,
   us.ihmc.valkyrie.pushRecovery.ValkyriePushRecoveryMultiStepTest.class
})

public class ValkyrieAInDevelopmentTestSuite
{
   public static void main(String[] args)
   {

   }
}
