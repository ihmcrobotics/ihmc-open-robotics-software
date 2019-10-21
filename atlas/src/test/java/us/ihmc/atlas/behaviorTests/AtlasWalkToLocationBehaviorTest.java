package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCWalkToLocationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasWalkToLocationBehaviorTest extends DRCWalkToLocationBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasWalkToLocationBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testTurn361DegreesInPlace() throws SimulationExceededMaximumTimeException
   {
      super.testTurn361DegreesInPlace();
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testWalkAndStopBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAndStopBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleAndFinishAlignedWithInitialOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleAndFinishAlignedWithWalkingPath() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleAndFinishAlignedWithWalkingPath();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleUsingStartOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingStartOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleUsingStartTargetMeanOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingStartTargetMeanOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleUsingTargetOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingTargetOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkBackwardsASmallAmountWithoutTurningInPlace() throws SimulationExceededMaximumTimeException
   {
      super.testWalkBackwardsASmallAmountWithoutTurningInPlace();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      super.testWalkForwardsX();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkPauseAndResumeBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeOnLastStepBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkPauseAndResumeOnLastStepBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Disabled
   @Test
   public void testWalkStopAndWalkToDifferentLocation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkStopAndWalkToDifferentLocation();
   }
}
