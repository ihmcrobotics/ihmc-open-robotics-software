package us.ihmc.atlas.stateEstimation;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator.PelvisPoseHistoryCorrectionTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPelvisPoseHistoryCorrectorTest extends PelvisPoseHistoryCorrectionTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
      AtlasPelvisPoseHistoryCorrectorTest test = new AtlasPelvisPoseHistoryCorrectorTest();
      test.setUp();
      test.runPelvisCorrectionControllerOutOfTheLoop();
   }
}


