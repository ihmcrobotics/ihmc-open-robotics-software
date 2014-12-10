package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.atlas.controllers.responses.AtlasHandPoseStatusTest;
import us.ihmc.atlas.stateEstimation.AtlasPelvisPoseHistoryCorrectorTest;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   AtlasBambooTestSuite.class,
   
   AtlasFlatGroundRewindabilityTest.class,
   AtlasBumpyAndShallowRampsWalkingTest.class,
   AtlasPushRecoveryTest.class,
   AtlasPushRecoveryStandingTest.class,
   AtlasPushRecoveryMultiStepTest.class,
   AtlasPelvisPoseHistoryCorrectorTest.class,
   AtlasHandPoseStatusTest.class
//   AtlasPlaybackPoseInterpolatorTest.class
})

public class AtlasBambooTestSuiteNightly
{
}
