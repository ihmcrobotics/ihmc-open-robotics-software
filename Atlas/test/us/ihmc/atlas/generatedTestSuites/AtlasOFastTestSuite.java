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
   us.ihmc.atlas.packets.AtlasRobotConfigurationDataTest.class,
   us.ihmc.atlas.posePlayback.AtlasPlaybackPoseSequenceTest.class,
   us.ihmc.atlas.pushRecovery.AtlasMomentumRecoveryTest.class,
   us.ihmc.atlas.roughTerrainWalking.AtlasEndToEndCinderBlockFieldTest.class,
   us.ihmc.atlas.roughTerrainWalking.AtlasFootstepSnapperTest.class
})

public class AtlasOFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
