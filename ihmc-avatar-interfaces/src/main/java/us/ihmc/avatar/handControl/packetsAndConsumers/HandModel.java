package us.ihmc.avatar.handControl.packetsAndConsumers;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;

public interface HandModel
{
	public HandJointName[] getHandJointNames();

}
