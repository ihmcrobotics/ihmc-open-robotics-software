package us.ihmc.alexander.rigidBodies;

import us.ihmc.alexander.AlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
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
      return new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }
}
