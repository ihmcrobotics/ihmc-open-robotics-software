package us.ihmc.atlas.stateEstimation;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.stateEstimationEndToEndTests.PelvisPoseHistoryCorrectionEndToEndTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class AtlasPelvisPoseHistoryCorrectorTest extends PelvisPoseHistoryCorrectionEndToEndTest
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
   @ContinuousIntegrationTest(estimatedDuration = 7.7)
   @Test(timeout = 30000)
   public void testBigYawInDoubleSupport() throws SimulationExceededMaximumTimeException
   {
      super.testBigYawInDoubleSupport();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 9.4)
   @Test(timeout = 30000)
   public void testBigYawInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      super.testBigYawInSingleSupport();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 8.1)
   @Test(timeout = 30000)
   public void testLocalizationOffsetOutsideOfFootInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      super.testLocalizationOffsetOutsideOfFootInSingleSupport();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 7.8)
   @Test(timeout = 30000)
   public void testPelvisCorrectionControllerOutOfTheLoop() throws SimulationExceededMaximumTimeException
   {
      super.testPelvisCorrectionControllerOutOfTheLoop();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 12.1)
   @Test(timeout = 30000)
   public void testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 5.0)
   @Test(timeout = 30000)
   public void testWalkingDuringBigPelvisCorrection() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testWalkingDuringBigPelvisCorrection();
   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
      AtlasPelvisPoseHistoryCorrectorTest test = new AtlasPelvisPoseHistoryCorrectorTest();
      test.setUp();
      test.runPelvisCorrectionControllerOutOfTheLoop();
   }
}


