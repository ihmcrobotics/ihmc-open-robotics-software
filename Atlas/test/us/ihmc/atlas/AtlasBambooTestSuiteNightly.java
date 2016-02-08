package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({ 
   us.ihmc.atlas.AtlasBumpyAndShallowRampsWalkingTest.class,
   us.ihmc.atlas.AtlasFlatGroundRewindabilityTest.class,
   us.ihmc.atlas.AtlasFlatGroundWalkingTest.class,
   us.ihmc.atlas.AtlasMultiContactTest.class,
   us.ihmc.atlas.AtlasPushRecoveryMultiStepTest.class,
   us.ihmc.atlas.AtlasPushRecoveryStandingTest.class,
   us.ihmc.atlas.AtlasPushRecoveryTest.class,
   us.ihmc.atlas.AtlasPushRecoveryWalkingTest.class,
   us.ihmc.atlas.behaviorTests.AtlasChestOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasComHeightBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasDetectCollisionUsingWristSensorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHandDesiredConfigurationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHandPoseBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHandPoseListBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHeadOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasLookAtBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasPelvisPoseBehaviorTest.class,
   us.ihmc.atlas.calib.KinematicCalibrationHeadLoopResidualTest.class,
   us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest.class,
   us.ihmc.atlas.controllers.responses.AtlasHandPoseStatusTest.class,
   us.ihmc.atlas.drcRobot.AtlasSDFVerificationTest.class,
   us.ihmc.atlas.HumanoidBehaviorsICPFaultDetectionTest.class,
   us.ihmc.atlas.momentumBasedControl.AtlasOptimizationMomentumControlModuleTest.class,
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
   us.ihmc.atlas.roughTerrainWalking.AtlasFootExplorationTest.class,
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

