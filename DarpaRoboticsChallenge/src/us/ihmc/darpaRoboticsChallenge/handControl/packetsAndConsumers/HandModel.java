package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import us.ihmc.communication.packets.dataobjects.HandJointName;

public interface HandModel
{
	public HandJointName[] getHandJointNames();

}
