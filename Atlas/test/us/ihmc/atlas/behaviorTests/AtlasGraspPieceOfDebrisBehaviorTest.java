package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCGraspPieceOfDebrisBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;


@BambooPlan(planType = {BambooPlanType.InDevelopment})
public class AtlasGraspPieceOfDebrisBehaviorTest extends DRCGraspPieceOfDebrisBehaviorTest 
{
	private final AtlasRobotModel robotModel;
	
	public AtlasGraspPieceOfDebrisBehaviorTest() 
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
	
	@Override
	@EstimatedDuration(duration = 70.0)
   @Test(timeout = 300000)
	public void testGraspingHorizontalDebrisWithRightHand() throws SimulationExceededMaximumTimeException
	{
	   super.testGraspingHorizontalDebrisWithRightHand();
	}
	
	@Override
	@EstimatedDuration(duration = 70.0)
   @Test(timeout = 300000)
	public void testGraspingLeaningAgainstAWallDebrisWithRightHand() throws SimulationExceededMaximumTimeException
	{
	   BambooPlanType.assumeRunningLocally();
	   super.testGraspingLeaningAgainstAWallDebrisWithRightHand();
	}
	
	@Override
	@EstimatedDuration(duration = 70.0)
   @Test(timeout = 300000)
	public void testGraspingStandingDebrisWithLeftHand() throws SimulationExceededMaximumTimeException
	{
	   super.testGraspingStandingDebrisWithLeftHand();
	}
	
	@Override
	@EstimatedDuration(duration = 70.0)
   @Test(timeout = 300000)
	public void testGraspingStandingDebrisWithRightHand() throws SimulationExceededMaximumTimeException
	{
	   super.testGraspingStandingDebrisWithRightHand();
	}
}
