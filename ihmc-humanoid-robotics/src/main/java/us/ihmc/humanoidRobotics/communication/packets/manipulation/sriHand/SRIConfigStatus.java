package us.ihmc.humanoidRobotics.communication.packets.manipulation.sriHand;

import us.ihmc.communication.packets.Packet;

public class SRIConfigStatus extends Packet<SRIConfigStatus>
{

	@Override
	public boolean epsilonEquals(SRIConfigStatus other, double epsilon)
	{
		return false;
	}

}
