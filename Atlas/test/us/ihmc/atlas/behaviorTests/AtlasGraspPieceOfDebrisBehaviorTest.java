package us.ihmc.atlas.behaviorTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCGraspPieceOfDebrisBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;

public class AtlasGraspPieceOfDebrisBehaviorTest extends DRCGraspPieceOfDebrisBehaviorTest 
{
	private final AtlasRobotModel robotModel;
	
	public AtlasGraspPieceOfDebrisBehaviorTest() 
	{
		robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
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
