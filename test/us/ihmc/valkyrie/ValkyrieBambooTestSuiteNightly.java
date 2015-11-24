package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
//   us.ihmc.valkyrie.codeGenerators.APIBuilderTest.class,
//   us.ihmc.valkyrie.networkProcessor.depthData.ValkyrieDepthDataProcessorTest.class,
//   us.ihmc.valkyrie.simulation.ValkyrieFlatGroundWalkingWithIMUDriftTest.class,
   us.ihmc.valkyrie.simulation.ValkyriePosePlaybackDemoTest.class,
   us.ihmc.valkyrie.ValkyrieFlatGroundWalkingTest.class,
   us.ihmc.valkyrie.ValkyrieObstacleCourseEveryBuildTest.class,
   us.ihmc.valkyrie.ValkyrieObstacleCourseFlatTest.class,
   us.ihmc.valkyrie.ValkyrieObstacleCourseRampsTest.class,
   us.ihmc.valkyrie.ValkyrieObstacleCourseTrialsTerrainTest.class,
   us.ihmc.valkyrie.ValkyriePushRecoveryMultiStepTest.class,
   us.ihmc.valkyrie.ValkyriePushRecoveryStandingTest.class,
   us.ihmc.valkyrie.ValkyriePushRecoveryTest.class,
//   us.ihmc.valkyrie.ValkyriePushRecoveryWalkingTest.class
})

public class ValkyrieBambooTestSuiteNightly
{
   public static void main(String[] args)
   {
   }
}

