package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FLAKY)
@SuiteClasses
({
   us.ihmc.atlas.AtlasFlatGroundRewindabilityTest.class,
   us.ihmc.atlas.behaviorTests.AtlasChestTrajectoryBehaviorTest.class,
   us.ihmc.atlas.communication.producers.AtlasRobotConfigurationDataBufferTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCoursePlatformTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseWobblyFootTest.class
})

public class AtlasAFlakyTestSuite
{
   public static void main(String[] args)
   {

   }
}
