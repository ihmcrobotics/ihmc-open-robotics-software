package us.ihmc.zulu.rigidBodies;

import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.rigidBodies.RigidBodyHashTest;

public class ZuluRigidBodyHashTest extends RigidBodyHashTest
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
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }
}
