package us.ihmc.atlas;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.AvatarDoubleStepTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;

public class AtlasDoubleStepTest extends AvatarDoubleStepTest
{
   private final RobotTarget target = RobotTarget.SCS;

   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, target, false);

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
