package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.EnumMap;

import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;

public abstract class DRCRobotPhysicalProperties
{
	public final EnumMap<ArmJointName, Pair<Double, Double>> armJointLimits = new EnumMap<ArmJointName, Pair<Double, Double>>(ArmJointName.class);
	
	public abstract double getAnkleHeight();
	public Pair<Double, Double> getArmJointMinAndMax(ArmJointName jointName)
	{
		return armJointLimits.get(jointName);
	}
}
