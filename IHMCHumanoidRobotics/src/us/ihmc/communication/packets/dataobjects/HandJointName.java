package us.ihmc.communication.packets.dataobjects;

import us.ihmc.humanoidRobotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public interface HandJointName
{
	public FingerName getFinger(RobotSide robotSide);

	public int getHandJointAngleIndex();
	
	public HandJointName[] getValues();

	public String getJointName(RobotSide robotSide);
}
