package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({ 
   us.ihmc.atlas.AtlasBumpyAndShallowRampsWalkingTest.class,
   us.ihmc.atlas.AtlasFlatGroundRewindabilityTest.class,
   us.ihmc.atlas.AtlasFlatGroundWalkingTest.class,
   us.ihmc.atlas.pushRecovery.AtlasPushRecoveryMultiStepTest.class,
   us.ihmc.atlas.pushRecovery.AtlasPushRecoveryStandingTest.class,
   us.ihmc.atlas.pushRecovery.AtlasPushRecoveryTest.class,
   us.ihmc.atlas.pushRecovery.AtlasPushRecoveryWalkingTest.class,
   us.ihmc.atlas.behaviorTests.AtlasChestTrajectoryBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHandDesiredConfigurationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHeadTrajectoryBehaviorTest.class,
   us.ihmc.atlas.calib.KinematicCalibrationHeadLoopResidualTest.class,
   us.ihmc.atlas.drcRobot.AtlasSDFVerificationTest.class,
   us.ihmc.atlas.HumanoidBehaviorsICPFaultDetectionTest.class,
   us.ihmc.atlas.networkProcessor.depthData.AtlasPointCloudDataReceiverTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasBigStepUpWithHandPlatformTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseEveryBuildTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseFlatTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCoursePlatformTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseRampsTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseRocksTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseStandingYawedTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseSteppingStonesTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseTrialsTerrainTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseTrialsWalkingTaskTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasWallWorldTest.class,
   us.ihmc.atlas.posePlayback.AtlasPlaybackPoseSequenceTest.class,
   us.ihmc.atlas.stateEstimation.AtlasPelvisPoseHistoryCorrectorTest.class,
   us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class,
   us.ihmc.atlas.XStreamTest.class
})

public class AtlasBambooTestSuiteNightly
{
   public static void main(String[] args)
   {
//      JUnitTestSuiteGenerator.generateTestSuite(AtlasBambooTestSuiteNightly.class);
   }
}

