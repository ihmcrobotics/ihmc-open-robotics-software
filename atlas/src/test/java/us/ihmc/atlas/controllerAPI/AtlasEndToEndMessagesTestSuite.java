package us.ihmc.atlas.controllerAPI;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import org.junit.runners.Suite.SuiteClasses;

@RunWith(Suite.class)
@SuiteClasses({AtlasEndToEndArmDesiredAccelerationsMessageTest.class, AtlasEndToEndArmTrajectoryMessageTest.class,
      AtlasEndToEndChestTrajectoryMessageTest.class, AtlasEndToEndFootLoadBearingTest.class, AtlasEndToEndFootTrajectoryMessageTest.class,
      AtlasEndToEndHandTrajectoryMessageTest.class, AtlasEndToEndHeadTrajectoryMessageTest.class, AtlasEndToEndPelvisHeightTrajectoryMessageTest.class,
      AtlasEndToEndPelvisTrajectoryMessageTest.class, AtlasEndToEndWholeBodyTrajectoryMessageTest.class})
public class AtlasEndToEndMessagesTestSuite
{

}
