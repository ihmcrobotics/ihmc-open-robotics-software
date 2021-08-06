package us.ihmc.valkyrie.footstepPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.AvatarReachabilityStepTest;
import us.ihmc.avatar.initialSetup.HumanoidRobotMutableInitialSetup;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.valkyrie.ValkyrieMutableInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilityStepTest extends AvatarReachabilityStepTest
{
   @Test
   @Override
   public void testSingleStep() throws Exception
   {
      super.testSingleStep();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Override
   protected HumanoidRobotMutableInitialSetup createInitialSetup(HumanoidJointNameMap jointNameMap)
   {
      return new ValkyrieMutableInitialSetup(jointNameMap);
   }
}
