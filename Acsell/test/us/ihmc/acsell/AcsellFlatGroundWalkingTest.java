package us.ihmc.acsell;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AcsellFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   @Test
   public void testBONOFlatGroundWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "BONOFlatGroundWalkingTest";
      BonoRobotModel robotModel = new BonoRobotModel();

      setupAndTestFlatGroundSimulationTrack(robotModel, runName);
   }
}
