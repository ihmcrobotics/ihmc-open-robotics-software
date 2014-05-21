package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ValkyrieFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{

   private DRCRobotModel robotModel;

   @Test
   public void testValkyrieFlatGroundWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "ValkyrieFlatGroundWalkingTest";
      robotModel = new ValkyrieRobotModel(false, false);

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
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
}
