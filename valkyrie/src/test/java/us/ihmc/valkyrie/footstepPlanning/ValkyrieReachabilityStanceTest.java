package us.ihmc.valkyrie.footstepPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.AvatarReachabilityStanceTest;
import us.ihmc.avatar.initialSetup.HumanoidRobotMutableInitialSetup;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.valkyrie.ValkyrieMutableInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilityStanceTest extends AvatarReachabilityStanceTest
{
   @Test
   @Override
   public void testStaticStances() throws Exception
   {
      super.testStaticStances();
   }

   @Override
   protected HumanoidRobotMutableInitialSetup createInitialSetup(HumanoidJointNameMap jointNameMap)
   {
      return new ValkyrieMutableInitialSetup(jointNameMap);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }
}
