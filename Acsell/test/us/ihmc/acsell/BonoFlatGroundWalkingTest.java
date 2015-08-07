package us.ihmc.acsell;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;

@BambooPlan(planType = BambooPlanType.InDevelopment)
public class BonoFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   private BonoRobotModel robotModel;

	@EstimatedDuration
	@Test(timeout=300000)
   public void testBONOFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
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
