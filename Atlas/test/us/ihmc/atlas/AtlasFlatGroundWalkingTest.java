package us.ihmc.atlas;

import org.junit.Assume;
import org.junit.Test;
import org.junit.internal.AssumptionViolatedException;

import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;

// This test is slow but very important, let's keep it in the FAST build please. (Sylvain)
@DeployableTestClass(planType = {BambooPlanType.Fast, BambooPlanType.VideoA})
public class AtlasFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   private DRCRobotModel robotModel;

	@DeployableTestMethod(duration = 208.3)
	@Test(timeout = 1000000)
   public void testAtlasFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "AtlasFlatGroundWalkingTest";
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);

      boolean doPelvisYawWarmup = true;
      setupAndTestFlatGroundSimulationTrack(robotModel, runName, doPelvisYawWarmup);
   }

	@DeployableTestMethod(duration = 0.3)
	@Test(timeout = 30000)
   public void testFlatGroundWalkingRunsSameWayTwice() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      try
      {
         Assume.assumeTrue(BambooTools.isNightlyBuild());
         BambooTools.reportTestStartedMessage();

         robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);

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
