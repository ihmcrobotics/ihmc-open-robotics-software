package us.ihmc.atlas;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.AvatarDoubleStepTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasDoubleStepTest extends AvatarDoubleStepTest
{
   private final RobotTarget target = RobotTarget.SCS;

   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, target, false);

   @Tag("humanoid-flat-ground-slow-2")
   @Test
   @Override
   public void testTwoStepsInARowSameSide() throws SimulationExceededMaximumTimeException
   {
      super.testTwoStepsInARowSameSide();
   }

   @Tag("allocation-slow")
   @Test
   @Override
   public void testTwoStepsInARowSameSideAfterFirstSep() throws SimulationExceededMaximumTimeException
   {
      super.testTwoStepsInARowSameSideAfterFirstSep();
   }

   @Tag("allocation-slow")
   @Test
   @Override
   public void testTwoStepsInARowLongTransferSameSide() throws SimulationExceededMaximumTimeException
   {
      super.testTwoStepsInARowLongTransferSameSide();
   }

   @Tag("humanoid-flat-ground-slow-2")
   @Test
   @Override
   public void testTwoStepsStandingInBetween() throws SimulationExceededMaximumTimeException
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
