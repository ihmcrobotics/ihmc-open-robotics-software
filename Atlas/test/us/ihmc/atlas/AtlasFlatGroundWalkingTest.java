package us.ihmc.atlas;

import org.junit.Assume;
import org.junit.Test;
import org.junit.internal.AssumptionViolatedException;

import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;

// This test is slow but very important, let's keep it in the FAST build please. (Sylvain)
@BambooPlan(planType = {BambooPlanType.Fast, BambooPlanType.VideoA})
public class AtlasFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   private DRCRobotModel robotModel;

	@EstimatedDuration(duration = 119.9)
	@Test(timeout = 1009754)
   public void testAtlasFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "AtlasFlatGroundWalkingTest";
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);

      boolean doPelvisYawWarmup = true;
      setupAndTestFlatGroundSimulationTrack(robotModel, runName, doPelvisYawWarmup);
   }

	@EstimatedDuration(duration = 0.7)
	@Test(timeout = 300000)
   public void testFlatGroundWalkingRunsSameWayTwice() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      try
      {
         Assume.assumeTrue(BambooTools.isNightlyBuild());
         BambooTools.reportTestStartedMessage();

         robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);

         setupAndTestFlatGroundSimulationTrackTwice(robotModel);
      }
      catch(AssumptionViolatedException e)
      {
         System.out.println("Not Nightly Build, skipping AtlasFlatGroundWalkingTest.testFlatGroundWalkingRunsSameWayTwice");
      }
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
