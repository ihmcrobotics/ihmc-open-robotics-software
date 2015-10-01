package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Flaky)
@SuiteClasses
({
   us.ihmc.atlas.behaviorTests.AtlasHandPoseListBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasPelvisPoseBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasScriptBehaviorTest.class,
   us.ihmc.atlas.communication.producers.AtlasRobotConfigurationDataBufferTest.class
})

public class AtlasDFlakyTestSuite
{
   public static void main(String[] args)
   {

   }
}
