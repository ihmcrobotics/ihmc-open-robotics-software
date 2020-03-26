package us.ihmc.valkyrie.simulation;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.HumanoidExperimentalSimulationEndToEndTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;

public class ValkyrieExperimentalSimulationEndToEndTest extends HumanoidExperimentalSimulationEndToEndTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Override
   public RobotCollisionModel getRobotCollisionModel(CollidableHelper helper, String robotCollisionMask, String... environmentCollisionMasks)
   {
      ValkyrieSimulationCollisionModel collisionModel = new ValkyrieSimulationCollisionModel(getRobotModel().getJointMap());
      collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
      return collisionModel;
   }

   @Test
   @Override
   public void testStanding(TestInfo testInfo) throws Exception
   {
      super.testStanding(testInfo);
   }

   @Test
   @Override
   public void testZeroTorque(TestInfo testInfo) throws Exception
   {
      super.testZeroTorque(testInfo);
   }
}
