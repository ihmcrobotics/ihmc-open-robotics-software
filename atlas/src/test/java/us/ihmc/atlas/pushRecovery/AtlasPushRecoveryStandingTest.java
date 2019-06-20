package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCPushRecoveryStandingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

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

	@Disabled("Needs to be improved")
   @Test
	@Override
	public void testDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testDoublePushForwardInDoubleSupportAndContinueWalking();
	}

   @Test
	@Override
	public void testPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushBackwardForwardInDoubleSupportAndContinueWalking();
	}

   @Test
	@Override
	public void testPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushBackwardInDoubleSupportAndContinueWalking();
	}

   @Test
	@Override
	public void testPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushForwardInDoubleSupport();
	}

   @Test
	@Override
	public void testPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
	{
	   super.testPushForwardInDoubleSupportAndContinueWalking();
	}
}
