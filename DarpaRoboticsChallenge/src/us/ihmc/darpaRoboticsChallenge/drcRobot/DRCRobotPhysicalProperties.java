package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.utilities.Pair;

public abstract class DRCRobotPhysicalProperties
{
	public static final EnumMap<ArmJointName, Pair<Double, Double>> armJointLimits = new EnumMap<ArmJointName, Pair<Double, Double>>(ArmJointName.class);
	
	public abstract double getAnkleHeight();
	public static Pair<Double, Double> getArmJointMinAndMax(ArmJointName jointName)
	{
		return armJointLimits.get(jointName);
	}
}
