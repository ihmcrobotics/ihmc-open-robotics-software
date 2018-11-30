package us.ihmc.atlas.roughTerrainWalking;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.DRCSwingTrajectoryTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
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
   @ContinuousIntegrationTest(estimatedDuration = 130.4)
   @Test(timeout = 650000)
   public void testMultipleHeightFootsteps() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleHeightFootsteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 47.8)
   @Test(timeout = 240000)
   public void testNegativeSwingHeight() throws SimulationExceededMaximumTimeException
   {
      super.testNegativeSwingHeight();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 55.8)
   @Test(timeout = 280000)
   public void testReallyHighFootstep() throws SimulationExceededMaximumTimeException
   {
      super.testReallyHighFootstep();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 126.9)
   @Test(timeout = 630000)
   public void testSelfCollisionAvoidance() throws SimulationExceededMaximumTimeException
   {
      super.testSelfCollisionAvoidance();
   }
}
