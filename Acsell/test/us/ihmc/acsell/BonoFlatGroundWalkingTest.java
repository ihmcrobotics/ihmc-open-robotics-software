package us.ihmc.acsell;

import org.junit.Test;

import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class BonoFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{

   private BonoRobotModel robotModel;

   @Test
   public void testBONOFlatGroundWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "BONOFlatGroundWalkingTest";
      robotModel = new BonoRobotModel(false, false);

      boolean doPelvisYawWarmup = false;
      setupAndTestFlatGroundSimulationTrack(robotModel, runName, doPelvisYawWarmup);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.BONO);
   }
}
