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
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndChestTrajectoryMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndEndEffectorLoadBearingMessageTest.class
})

public class ValkyrieBFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
