package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.AvatarCollaborativeBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasCollaborativeTaskTest extends AvatarCollaborativeBehaviorTest{

	AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
	AtlasRobotModel robotModel = new AtlasRobotModel(version, RobotTarget.SCS, false);

	@Override
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 30000)
	public void testBehavior() throws SimulationExceededMaximumTimeException
	{
		super.testBehavior();
	}

	@Override
	public DRCRobotModel getRobotModel() {
		return robotModel;
	}

	@Override
	public String getSimpleRobotName() {
		return robotModel.getSimpleRobotName();
	}

}
