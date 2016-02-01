package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;

public interface HandModel
{
	public HandJointName[] getHandJointNames();

}
