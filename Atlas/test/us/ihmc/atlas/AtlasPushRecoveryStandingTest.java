package us.ihmc.atlas;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryStandingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.Slow, TestPlanTarget.VideoA})
public class AtlasPushRecoveryStandingTest extends DRCPushRecoveryStandingTest {

	@Override
	public DRCRobotModel getRobotModel() 
	{
		return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
	}

	@Override
	public String getSimpleRobotName() 
	{
		return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
	}
	
	@Override
	@DeployableTestMethod(estimatedDuration = 49.1)
   @Test(timeout = 250000)
	public void testDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testDoublePushForwardInDoubleSupportAndContinueWalking();
	}
	
	@Override
	@DeployableTestMethod(estimatedDuration = 52.2)
   @Test(timeout = 260000)
	public void testPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushBackwardForwardInDoubleSupportAndContinueWalking();
	}
	
	@Override
	@DeployableTestMethod(estimatedDuration = 44.8)
   @Test(timeout = 220000)
	public void testPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushBackwardInDoubleSupportAndContinueWalking();
	}
	
	@Override
	@DeployableTestMethod(estimatedDuration = 30.6)
   @Test(timeout = 150000)
	public void testPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushForwardInDoubleSupport();
	}
	
	@Override
	@DeployableTestMethod(estimatedDuration = 46.6)
   @Test(timeout = 230000)
	public void testPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushForwardInDoubleSupportAndContinueWalking();
	}
}
