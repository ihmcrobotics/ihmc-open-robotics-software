package us.ihmc.atlas.pushRecovery;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCPushRecoveryStandingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class AtlasPushRecoveryStandingTest extends DRCPushRecoveryStandingTest
{
   @Override
	public DRCRobotModel getRobotModel()
	{
		return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
	}

	@Override
	public String getSimpleRobotName()
	{
		return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
	}

	@Ignore("Needs to be improved")
	@ContinuousIntegrationTest(estimatedDuration = 53.2, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 270000)
	@Override
	public void testDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testDoublePushForwardInDoubleSupportAndContinueWalking();
	}

	@ContinuousIntegrationTest(estimatedDuration = 68.4)
   @Test(timeout = 340000)
	@Override
	public void testPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushBackwardForwardInDoubleSupportAndContinueWalking();
	}

	@ContinuousIntegrationTest(estimatedDuration = 56.4)
   @Test(timeout = 280000)
	@Override
	public void testPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushBackwardInDoubleSupportAndContinueWalking();
	}

	@ContinuousIntegrationTest(estimatedDuration = 39.3)
   @Test(timeout = 200000)
	@Override
	public void testPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushForwardInDoubleSupport();
	}

	@ContinuousIntegrationTest(estimatedDuration = 61.3)
   @Test(timeout = 310000)
	@Override
	public void testPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushForwardInDoubleSupportAndContinueWalking();
	}
}
