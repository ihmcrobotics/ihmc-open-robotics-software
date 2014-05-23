package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.atlas.posePlayback.AtlasPlaybackPoseInterpolatorTest;


@RunWith(Suite.class)
@Suite.SuiteClasses({
   us.ihmc.atlas.AtlasFlatGroundRewindabilityTest.class,
   us.ihmc.atlas.AtlasBumpyAndShallowRampsWalkingTest.class,
   us.ihmc.atlas.AtlasPushInSingleSupportTest.class,
//   AtlasPlaybackPoseInterpolatorTest.class
})
public class AtlasBambooTestSuiteNightly
{
}
