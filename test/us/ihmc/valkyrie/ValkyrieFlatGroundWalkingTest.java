package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;

//This test is slow but very important, let's keep it in the FAST build please. (Sylvain)
@BambooPlan(planType = {BambooPlanType.Fast, BambooPlanType.VideoB})
public class ValkyrieFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{

   private DRCRobotModel robotModel;

	@EstimatedDuration(duration = 144.4)
	@Test(timeout = 433159)
   public void testValkyrieFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "ValkyrieFlatGroundWalkingTest";
      robotModel = new ValkyrieRobotModel(false, false);

      boolean doPelvisYawWarmup = true;
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
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
}
