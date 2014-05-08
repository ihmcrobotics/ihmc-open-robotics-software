package us.ihmc.acsell;

import org.junit.Test;

import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class BonoFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   @Test
   public void testBONOFlatGroundWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "BONOFlatGroundWalkingTest";
      BonoRobotModel robotModel = new BonoRobotModel(false, false);

      setupAndTestFlatGroundSimulationTrack(robotModel, runName);
   }
}
