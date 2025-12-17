package us.ihmc.openAlexander.rigidBodies;

import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.rigidBodies.RigidBodyHashTest;

public class AlexanderRigidBodyHashTest extends RigidBodyHashTest
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
      return new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }
}
