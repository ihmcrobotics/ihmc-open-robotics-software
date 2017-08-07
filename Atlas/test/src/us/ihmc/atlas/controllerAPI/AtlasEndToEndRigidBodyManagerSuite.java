package us.ihmc.atlas.controllerAPI;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   AtlasEndToEndHandTrajectoryMessageTest.class,
   AtlasEndToEndArmTrajectoryMessageTest.class,
   AtlasEndToEndArmDesiredAccelerationsMessageTest.class,
   AtlasEndToEndChestTrajectoryMessageTest.class,
   AtlasEndToEndSpineJointTrajectoryMessageTest.class,
   AtlasEndToEndChestDesiredAccelerationsMessage.class,
   AtlasEndToEndHeadTrajectoryMessageTest.class,
   AtlasEndToEndWholeBodyTrajectoryMessageTest.class
})

public class AtlasEndToEndRigidBodyManagerSuite
{

}
