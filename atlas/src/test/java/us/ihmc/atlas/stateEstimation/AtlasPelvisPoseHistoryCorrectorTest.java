package us.ihmc.atlas.stateEstimation;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.stateEstimationEndToEndTests.PelvisPoseHistoryCorrectionEndToEndTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Disabled
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
   @Test
   public void testBigYawInDoubleSupport() throws SimulationExceededMaximumTimeException
   {
      super.testBigYawInDoubleSupport();
   }

   @Override
   @Test
   public void testBigYawInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      super.testBigYawInSingleSupport();
   }

   @Override
   @Test
   public void testLocalizationOffsetOutsideOfFootInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      super.testLocalizationOffsetOutsideOfFootInSingleSupport();
   }

   @Override
   @Test
   public void testPelvisCorrectionControllerOutOfTheLoop() throws SimulationExceededMaximumTimeException
   {
      super.testPelvisCorrectionControllerOutOfTheLoop();
   }

   @Override
   @Test
   public void testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Override
   @Test
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


