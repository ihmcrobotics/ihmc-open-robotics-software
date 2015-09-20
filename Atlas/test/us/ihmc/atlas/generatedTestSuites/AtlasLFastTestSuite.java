package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses
({
   us.ihmc.atlas.packets.wholebody.AtlasArmJointTrajectoryPacketTest.class,
   us.ihmc.atlas.packets.wholebody.AtlasWholeBodyTrajectoryPacketTest.class,
   us.ihmc.atlas.posePlayback.AtlasPlaybackPoseSequenceTest.class,
   us.ihmc.atlas.roughTerrainWalking.AtlasFootstepSnapperTest.class,
   us.ihmc.atlas.roughTerrainWalking.AtlasSwingTrajectoryTest.class,
   us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class
})

public class AtlasLFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
