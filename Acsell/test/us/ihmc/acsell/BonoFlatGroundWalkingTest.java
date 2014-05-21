package us.ihmc.acsell;

import org.junit.Test;

import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class BonoFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{

   private BonoRobotModel robotModel;

   @Test
   public void testBONOFlatGroundWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "BONOFlatGroundWalkingTest";
      robotModel = new BonoRobotModel(false, false);

      setupAndTestFlatGroundSimulationTrack(robotModel, runName);
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
