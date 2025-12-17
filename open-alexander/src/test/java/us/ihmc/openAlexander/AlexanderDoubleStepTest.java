package us.ihmc.openAlexander;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.AvatarDoubleStepTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;

public class AlexanderDoubleStepTest extends AvatarDoubleStepTest
{
   private final RobotTarget target = RobotTarget.SCS;

   private final OpenAlexanderRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, target);

   @Tag("humanoid-flat-ground-slow-2")
   @Test
   @Override
   public void testTwoStepsInARowSameSide() throws Exception
   {
      super.testTwoStepsInARowSameSide();
   }

   @Tag("allocation-slow")
   @Test
   @Override
   public void testTwoStepsInARowSameSideAfterFirstSep() throws Exception
   {
      super.testTwoStepsInARowSameSideAfterFirstSep();
   }

   @Tag("allocation-slow")
   @Test
   @Override
   public void testTwoStepsInARowLongTransferSameSide() throws Exception
   {
      super.testTwoStepsInARowLongTransferSameSide();
   }

   @Tag("humanoid-flat-ground-slow-2")
   @Test
   @Override
   public void testTwoStepsStandingInBetween() throws Exception
   {
      super.testTwoStepsStandingInBetween();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }
}
