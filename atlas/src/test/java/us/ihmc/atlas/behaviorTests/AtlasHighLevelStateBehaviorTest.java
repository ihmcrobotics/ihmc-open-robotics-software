package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCHighLevelStateBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasHighLevelStateBehaviorTest extends DRCHighLevelStateBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHighLevelStateBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
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

   @Override
   @Test
   public void testWalkingState() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingState();
   }

   @Override
   @Test
   public void testDoNothingBahviourState() throws SimulationExceededMaximumTimeException
   {
      super.testDoNothingBahviourState();
   }

   @Override
   @Disabled
   @Test
   public void testDiagnosticsState() throws SimulationExceededMaximumTimeException
   {
      super.testDiagnosticsState();
   }
}
