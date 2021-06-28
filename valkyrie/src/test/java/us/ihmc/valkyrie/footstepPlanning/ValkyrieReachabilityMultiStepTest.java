package us.ihmc.valkyrie.footstepPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.AvatarReachabilityMultiStepTest;
import us.ihmc.avatar.initialSetup.HumanoidRobotMutableInitialSetup;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.valkyrie.ValkyrieMutableInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilityMultiStepTest extends AvatarReachabilityMultiStepTest
{
   @Test
   @Override
   public void testMultipleSteps() throws Exception
   {
      super.testMultipleSteps();
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
