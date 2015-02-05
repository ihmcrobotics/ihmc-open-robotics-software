package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.networkProcessor.depthData.AtlasDepthDataProcessorTest.class,
   us.ihmc.atlas.networkProcessor.downlinkSerializers.AtlasJointConfigurationDataSerializerTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasBigStepUpWithHandPlatformTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseDoNothingTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseEveryBuildTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseFlatTest.class
})

public class AtlasJFastTestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(AtlasJFastTestSuite.class);
   }
}

