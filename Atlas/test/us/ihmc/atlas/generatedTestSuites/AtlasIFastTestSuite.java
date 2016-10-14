package us.ihmc.atlas.generatedTestSuites;

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
   us.ihmc.atlas.controllerAPI.AtlasEndToEndHandTrajectoryMessageTest.class,
   us.ihmc.atlas.controllerAPI.AtlasEndToEndHeadTrajectoryMessageTest.class,
   us.ihmc.atlas.controllerAPI.AtlasEndToEndPelvisHeightTrajectoryMessageTest.class,
   us.ihmc.atlas.controllerAPI.AtlasEndToEndPelvisTrajectoryMessageTest.class,
   us.ihmc.atlas.controllerAPI.AtlasEndToEndWholeBodyTrajectoryMessageTest.class,
   us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest.class,
   us.ihmc.atlas.initialSetup.AtlasDrivingInitialSetupTest.class
})

public class AtlasIFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
