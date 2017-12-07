package us.ihmc.humanoidRobotics.communication.packets.manipulation.sriHand;

import us.ihmc.communication.packets.Packet;

public class SRIHandStatus extends Packet<SRIHandStatus>
{

	@Override
	public boolean epsilonEquals(SRIHandStatus other, double epsilon)
	{
		return false;
	}

}
