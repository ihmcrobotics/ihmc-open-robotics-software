package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;


@RunWith(Suite.class)
@Suite.SuiteClasses({
   us.ihmc.atlas.AtlasFlatGroundRewindabilityTest.class,
   us.ihmc.atlas.AtlasBumpyAndShallowRampsWalkingTest.class,
   us.ihmc.atlas.AtlasPushRecoveryTest.class,
   us.ihmc.atlas.AtlasPushRecoveryStandingTest.class,
   us.ihmc.atlas.AtlasPushRecoveryMultiStepTest.class,
   us.ihmc.atlas.stateEstimation.AtlasPelvisPoseHistoryCorrectorTest.class,
   us.ihmc.atlas.controllers.responses.AtlasHandPoseStatusTest.class
//   AtlasPlaybackPoseInterpolatorTest.class
})
public class AtlasBambooTestSuiteNightly
{
}
