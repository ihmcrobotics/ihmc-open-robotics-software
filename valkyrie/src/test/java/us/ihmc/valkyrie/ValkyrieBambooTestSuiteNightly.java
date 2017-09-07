package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
//   us.ihmc.valkyrie.codeGenerators.APIBuilderTest.class,
//   us.ihmc.valkyrie.networkProcessor.depthData.ValkyrieDepthDataProcessorTest.class,
//   us.ihmc.valkyrie.simulation.ValkyrieFlatGroundWalkingWithIMUDriftTest.class,
   us.ihmc.valkyrie.simulation.ValkyrieFlatGroundWalkingTest.class,
   us.ihmc.valkyrie.obstacleCourse.ValkyrieObstacleCourseEveryBuildTest.class,
   us.ihmc.valkyrie.obstacleCourse.ValkyrieObstacleCourseFlatTest.class,
   us.ihmc.valkyrie.obstacleCourse.ValkyrieObstacleCourseRampsTest.class,
   us.ihmc.valkyrie.obstacleCourse.ValkyrieObstacleCourseTrialsTerrainTest.class,
   us.ihmc.valkyrie.pushRecovery.ValkyriePushRecoveryMultiStepTest.class,
   us.ihmc.valkyrie.pushRecovery.ValkyriePushRecoveryStandingTest.class,
   us.ihmc.valkyrie.pushRecovery.ValkyriePushRecoveryTest.class,
//   us.ihmc.valkyrie.ValkyriePushRecoveryWalkingTest.class
})

public class ValkyrieBambooTestSuiteNightly
{
   public static void main(String[] args)
   {
   }
}

