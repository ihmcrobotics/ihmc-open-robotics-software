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
   @ContinuousIntegrationTest(estimatedDuration = 105.2)
   @Test(timeout = 530000)
   public void testMultipleHeightFootsteps() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleHeightFootsteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 38.7)
   @Test(timeout = 190000)
   public void testNegativeSwingHeight() throws SimulationExceededMaximumTimeException
   {
      super.testNegativeSwingHeight();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 50.4)
   @Test(timeout = 250000)
   public void testReallyHighFootstep() throws SimulationExceededMaximumTimeException
   {
      super.testReallyHighFootstep();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 108.3)
   @Test(timeout = 540000)
   public void testSelfCollisionAvoidance() throws SimulationExceededMaximumTimeException
   {
      super.testSelfCollisionAvoidance();
   }
}
