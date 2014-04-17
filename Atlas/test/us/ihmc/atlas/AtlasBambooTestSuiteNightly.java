package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.atlas.posePlayback.AtlasPlaybackPoseInterpolatorTest;


@RunWith(Suite.class)
@Suite.SuiteClasses({
   us.ihmc.atlas.AtlasFlatGroundRewindabilityTest.class,
   us.ihmc.atlas.AtlasBumpyAndShallowRampsWalkingTest.class,
   AtlasPlaybackPoseInterpolatorTest.class
})
public class AtlasBambooTestSuiteNightly
{
}
