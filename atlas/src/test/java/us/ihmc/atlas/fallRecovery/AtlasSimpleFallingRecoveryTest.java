package us.ihmc.atlas.fallRecovery;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.fallRecovery.SimpleFallingRecoveryTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasSimpleFallingRecoveryTest extends SimpleFallingRecoveryTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Test
   @Override
   public void testFallingStateTriggered() throws SimulationExceededMaximumTimeException
   {
      super.testFallingStateTriggered();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }
}
