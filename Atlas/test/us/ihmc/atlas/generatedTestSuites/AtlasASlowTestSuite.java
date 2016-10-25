package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.SLOW)
@SuiteClasses
({
   us.ihmc.atlas.behaviorTests.AtlasBehaviorDispatcherTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHandDesiredConfigurationBehaviorTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasFinalsWorldStairsTest.class
})

public class AtlasASlowTestSuite
{
   public static void main(String[] args)
   {

   }
}
