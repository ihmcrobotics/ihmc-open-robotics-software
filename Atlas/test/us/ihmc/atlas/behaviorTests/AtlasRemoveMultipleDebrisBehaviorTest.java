package us.ihmc.atlas.behaviorTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCRemoveMultipleDebrisBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType = {BambooPlanType.InDevelopment})
public class AtlasRemoveMultipleDebrisBehaviorTest extends DRCRemoveMultipleDebrisBehaviorTest 
{
	private final AtlasRobotModel robotModel;
	
	public AtlasRemoveMultipleDebrisBehaviorTest() 
	{
		robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V4_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
		robotModel.getContactPointParameters().createHandContactPoints(false);
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
