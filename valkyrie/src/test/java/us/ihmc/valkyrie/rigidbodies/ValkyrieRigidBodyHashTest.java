package us.ihmc.valkyrie.rigidbodies;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.rigidbodies.RigidBodyHashTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieRigidBodyHashTest extends RigidBodyHashTest
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
      return new ValkyrieRobotModel(RobotTarget.SCS, true);
   }
}
