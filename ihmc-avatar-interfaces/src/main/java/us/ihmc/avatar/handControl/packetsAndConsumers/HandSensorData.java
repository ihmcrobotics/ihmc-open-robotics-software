package us.ihmc.avatar.handControl.packetsAndConsumers;

import us.ihmc.robotics.robotSide.RobotSide;

public interface HandSensorData
{
	public double[][] getFingerJointAngles(RobotSide robotSide);
	public boolean isCalibrated();
	public boolean isConnected();
}
