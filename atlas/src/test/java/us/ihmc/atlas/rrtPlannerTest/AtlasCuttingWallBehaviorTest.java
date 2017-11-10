package us.ihmc.atlas.rrtPlannerTest;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.wholeBodyPlanningTest.AvatarCuttingWallBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasCuttingWallBehaviorTest extends AvatarCuttingWallBehaviorTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);

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
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 400.0)
   @Test(timeout = 430000)
   public void testForCWBPlanningBehavior() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForCWBPlanningBehavior();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 300.0)
   @Test(timeout = 330000)
   public void testForCuttingWallBehavior() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForCuttingWallBehavior();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 72000)
   public void testForReachability() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForReachability();
   }
}