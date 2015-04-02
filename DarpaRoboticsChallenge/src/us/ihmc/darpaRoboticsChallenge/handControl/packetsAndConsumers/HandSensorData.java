package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import us.ihmc.utilities.robotSide.RobotSide;

public interface HandSensorData
{
	public double[][] getFingerJointAngles(RobotSide robotSide);

}
