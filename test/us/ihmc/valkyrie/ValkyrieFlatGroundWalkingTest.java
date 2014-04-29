package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ValkyrieFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   @Test
   public void testValkyrieFlatGroundWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "ValkyrieFlatGroundWalkingTest";
      DRCRobotModel robotModel = new ValkyrieRobotModel(false);

      setupAndTestFlatGroundSimulationTrack(robotModel, runName);
   }
}
