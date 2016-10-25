package us.ihmc.atlas.generatedTestSuites;

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
   us.ihmc.atlas.behaviorTests.AtlasHighLevelStateBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasWalkToLocationBehaviorTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasBigStepUpWithHandPlatformTest.class
})

public class AtlasBInDevelopmentTestSuite
{
   public static void main(String[] args)
   {

   }
}
