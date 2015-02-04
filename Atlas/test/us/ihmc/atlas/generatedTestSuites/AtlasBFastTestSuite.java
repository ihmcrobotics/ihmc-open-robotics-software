package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.AtlasScriptingTest.class,
   us.ihmc.atlas.behaviorTests.AtlasBehaviorDispatcherTest.class,
   us.ihmc.atlas.behaviorTests.AtlasChestOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasComHeightBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasDetectCollisionUsingWristSensorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasFingerStateBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasFootPoseBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasFootstepListBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasGraspPieceOfDebrisBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHandPoseBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHandPoseListBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHeadOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHighLevelStateBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasLookAtBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasPelvisPoseBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasScriptBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasTurnValveBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasWalkToGoalBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasWalkToLocationBehaviorTest.class,
   us.ihmc.atlas.calib.KinematicCalibrationHeadLoopResidualTest.class,
   us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest.class,
   us.ihmc.atlas.controllers.responses.AtlasHandPoseStatusTest.class,
   us.ihmc.atlas.drcRobot.AtlasSDFVerificationTest.class,
   us.ihmc.atlas.hikSim.AtlasWholeBodyIkSolverTest.class,
   us.ihmc.atlas.hikSim.AtlasWholeBodyTrajectoryTest.class,
   us.ihmc.atlas.HumanoidBehaviorsICPFaultDetectionTest.class,
   us.ihmc.atlas.momentumBasedControl.AtlasOptimizationMomentumControlModuleTest.class,
   us.ihmc.atlas.networking.AtlasNetworkingCommandReaderTest.class,
   us.ihmc.atlas.networkProcessor.depthData.AtlasDepthDataProcessorTest.class,
   us.ihmc.atlas.networkProcessor.downlinkSerializers.AtlasJointConfigurationDataSerializerTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasBigStepUpWithHandPlatformTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseDoNothingTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseEveryBuildTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseFlatTest.class
})

public class AtlasBFastTestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(AtlasBFastTestSuite.class);
   }
}

