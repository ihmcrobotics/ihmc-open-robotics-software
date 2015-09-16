package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.SdfLoader.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public interface HandJointName
{
	public FingerName getFinger(RobotSide robotSide);

	public int getHandJointAngleIndex();
	
	public HandJointName[] getValues();

	public String getJointName(RobotSide robotSide);
}
