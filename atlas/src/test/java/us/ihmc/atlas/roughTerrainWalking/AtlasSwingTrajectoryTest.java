package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.DRCSwingTrajectoryTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasSwingTrajectoryTest extends DRCSwingTrajectoryTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testMultipleHeightFootsteps() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleHeightFootsteps();
   }

   @Override
   @Test
   public void testNegativeSwingHeight() throws SimulationExceededMaximumTimeException
   {
      super.testNegativeSwingHeight();
   }

   @Override
   @Test
   public void testReallyHighFootstep() throws SimulationExceededMaximumTimeException
   {
      super.testReallyHighFootstep();
   }

   @Override
   @Test
   public void testSelfCollisionAvoidance() throws SimulationExceededMaximumTimeException
   {
      super.testSelfCollisionAvoidance();
   }
}
