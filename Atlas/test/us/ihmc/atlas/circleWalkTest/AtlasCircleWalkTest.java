package us.ihmc.atlas.circleWalkTest;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.circleWalkTest.HumanoidCircleWalkTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.avatar.drcRobot.NewRobotPhysicalProperties;
import us.ihmc.robotics.partNames.ArmJointName;

public class AtlasCircleWalkTest extends HumanoidCircleWalkTest{

	private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
	private final DRCRobotModel robotModel = new AtlasRobotModel(version, RobotTarget.SCS, false);
	private final AtlasJointMap jointMap = new AtlasJointMap(version, (NewRobotPhysicalProperties) robotModel.getPhysicalProperties());

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
		return 0.50;
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
