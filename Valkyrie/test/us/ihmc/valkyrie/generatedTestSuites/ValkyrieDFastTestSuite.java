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
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndHandTrajectoryMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndHeadTrajectoryMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndNeckDesiredAccelerationsMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndNeckTrajectoryMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndPelvisHeightTrajectoryMessageTest.class,
   us.ihmc.valkyrie.controllerAPI.ValkyrieEndToEndPelvisTrajectoryMessageTest.class
})

public class ValkyrieDFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
