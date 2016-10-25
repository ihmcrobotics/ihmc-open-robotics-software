package us.ihmc.valkyrie.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndArmDesiredAccelerationsMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndArmTrajectoryMessageTest.class
})

public class ValkyrieAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
