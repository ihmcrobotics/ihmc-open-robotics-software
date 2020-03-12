package us.ihmc.valkyrie.simulation;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.HumanoidRagdollTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieRagdollTest extends HumanoidRagdollTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Test
   @Override
   public void testZeroTorque(TestInfo testInfo) throws Exception
   {
      super.testZeroTorque(testInfo);
   }
}
