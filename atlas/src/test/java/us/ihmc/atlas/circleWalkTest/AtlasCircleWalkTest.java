package us.ihmc.atlas.circleWalkTest;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.circleWalkTest.HumanoidCircleWalkTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasCircleWalkTest extends HumanoidCircleWalkTest{

	private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
	private final AtlasRobotModel robotModel = new AtlasRobotModel(version, RobotTarget.SCS, false);
	private final AtlasJointMap jointMap = new AtlasJointMap(version, robotModel.getPhysicalProperties());

	@Override
   public void testCircleWalk() throws SimulationExceededMaximumTimeException
   {
      super.testCircleWalk();
   }

	@Override
	public DRCRobotModel getRobotModel()
	{
		return robotModel;
	}

	@Override
	public String getSimpleRobotName()
	{
		return robotModel.getSimpleRobotName();
	}

	@Override
	public double getRadiusForCircle()
	{
		return 2.00;
	}

	@Override
	public double getStepWidth()
	{
		return 0.35;
	}

	@Override
	public double getStepLength()
	{
		return 0.5;
	}

	@Override
	public int getArmDoF()
	{
		return jointMap.getArmJointNames().length;
	}

	@Override
	public ArmJointName[] getArmJointNames()
	{
		return jointMap.getArmJointNames();
	}
}
