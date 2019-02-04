package us.ihmc.atlas.rigidbodies;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.rigidbodies.RigidBodyHashTest;

public class AtlasRigidBodyHashTest extends RigidBodyHashTest
{
   @Override
   public void testSignificantRigidBodiesHashCode()
   {
      super.testSignificantRigidBodiesHashCode();
   }

   @Override
   public void testAllRigidBodiesHashCode()
   {
      super.testAllRigidBodiesHashCode();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true);
   }
}
