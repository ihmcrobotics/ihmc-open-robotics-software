package us.ihmc.atlas.behaviorTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCRemoveSingleDebrisBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;

public class AtlasRemoveSingleDebrisBehaviorTest extends DRCRemoveSingleDebrisBehaviorTest 
{
	private final AtlasRobotModel robotModel;
	
	public AtlasRemoveSingleDebrisBehaviorTest() 
	{
		robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
		robotModel.createHandContactPoints();
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
