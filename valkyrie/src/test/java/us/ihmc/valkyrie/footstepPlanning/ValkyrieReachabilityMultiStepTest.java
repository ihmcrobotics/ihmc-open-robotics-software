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
   public void testFlatForwards() throws Exception
   {
      super.testFlatForwards();
   }

   @Test
   @Override
   public void testFlatBackwards() throws Exception
   {
      super.testFlatBackwards();
   }

   @Test
   @Override
   public void testFlatLeft() throws Exception
   {
      super.testFlatLeft();
   }

   @Test
   @Override
   public void testFlatRight() throws Exception
   {
      super.testFlatRight();
   }

   @Test
   @Override
   public void testFlatRandom() throws Exception
   {
      super.testFlatRandom();
   }

   @Test
   @Override
   public void testStairsForwards() throws Exception
   {
      super.testStairsForwards();
   }

   @Test
   @Override
   public void testStairsBackwards() throws Exception
   {
      super.testStairsBackwards();
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
