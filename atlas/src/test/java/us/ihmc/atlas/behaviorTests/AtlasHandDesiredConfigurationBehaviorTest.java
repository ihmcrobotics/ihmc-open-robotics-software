package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.HumanoidHandDesiredConfigurationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasHandDesiredConfigurationBehaviorTest extends HumanoidHandDesiredConfigurationBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHandDesiredConfigurationBehaviorTest()
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
   @ContinuousIntegrationTest(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testCloseHand() throws SimulationExceededMaximumTimeException
   {
      super.testCloseHand();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 51.9)
   @Test(timeout = 260000)
   public void testPauseAndResumeCloseHand() throws SimulationExceededMaximumTimeException
   {
      super.testPauseAndResumeCloseHand();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 39.8)
   @Test(timeout = 200000)
   public void testStopCloseHand() throws SimulationExceededMaximumTimeException
   {
      super.testStopCloseHand();
   }
}
