package us.ihmc.communication.packets.wholebody;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.tools.random.RandomTools;

import java.util.Random;

@ClassDocumentation("This message contains wraps up multiple SingleJointAnglePackets so they will be"
		+ "executed simultaneusly.")
public class MultiJointAnglePacket extends IHMCRosApiPacket<MultiJointAnglePacket> implements VisualizablePacket
{
	public SingleJointAnglePacket[] singleJointAnglePackets;
	
	public MultiJointAnglePacket()
	{
	}

	public MultiJointAnglePacket(Random random)
	{
		int randomLength = random.nextInt(30) + 1;
		singleJointAnglePackets = new SingleJointAnglePacket[randomLength];

		for(int i = 0; i < randomLength; i++)
		{
			singleJointAnglePackets[i] = new SingleJointAnglePacket("testJoint" + i, RandomTools.generateRandomDouble(random, -5.0, 5.0), RandomTools.generateRandomDouble(random, 0.0, 10.0), Double.NaN);
		}
	}
	
	public MultiJointAnglePacket(SingleJointAnglePacket[] singleJointAnglePackets)
	{
		this.singleJointAnglePackets = singleJointAnglePackets;
	}

	@Override
	public boolean epsilonEquals(MultiJointAnglePacket other, double epsilon)
	{
		if (singleJointAnglePackets.length != other.singleJointAnglePackets.length)
		{
			return false;
		}
		
		for (int i = 0; i < singleJointAnglePackets.length; i++)
		{
			if (!singleJointAnglePackets[i].epsilonEquals(other.singleJointAnglePackets[i], epsilon))
			{
				return false;
			}
		}
		
		return true;
	}

}
