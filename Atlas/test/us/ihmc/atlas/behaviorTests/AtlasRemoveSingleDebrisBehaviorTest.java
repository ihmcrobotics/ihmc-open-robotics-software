package us.ihmc.atlas.behaviorTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCRemoveSingleDebrisBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;


@DeployableTestClass(planType = {BambooPlanType.InDevelopment})
public class AtlasRemoveSingleDebrisBehaviorTest extends DRCRemoveSingleDebrisBehaviorTest 
{
	private final AtlasRobotModel robotModel;
	
	public AtlasRemoveSingleDebrisBehaviorTest() 
	{
		robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
      boolean useHighResolutionContactPointGrid = false;
		robotModel.createHandContactPoints(useHighResolutionContactPointGrid);
	}

	@Override
	public DRCRobotModel getRobotModel() {
	      return robotModel;
	}

	@Override
	public String getSimpleRobotName()
	{
		return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
	}
}
