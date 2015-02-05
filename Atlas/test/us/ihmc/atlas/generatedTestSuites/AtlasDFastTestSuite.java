package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
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
   us.ihmc.atlas.hikSim.AtlasWholeBodyIkSolverTest.class
})

public class AtlasDFastTestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(AtlasDFastTestSuite.class);
   }
}

