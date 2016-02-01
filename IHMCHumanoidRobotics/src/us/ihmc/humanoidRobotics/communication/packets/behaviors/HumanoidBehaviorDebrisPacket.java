package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class HumanoidBehaviorDebrisPacket extends Packet<HumanoidBehaviorDebrisPacket> 
{
	public ArrayList<DebrisData> debrisDataList;
	
	public HumanoidBehaviorDebrisPacket(Random random)
	{
	   debrisDataList = new ArrayList<DebrisData>();
      int size = Math.abs(random.nextInt(1000));

      for (int i = 0; i < size; i++)
      {
         DebrisData debrisData = new DebrisData(random);
         debrisDataList.add(debrisData);
      }
	}
	
	public HumanoidBehaviorDebrisPacket()
	{
	}
	
	public HumanoidBehaviorDebrisPacket(ArrayList<DebrisData> debrisData)
	{
		this.debrisDataList = debrisData;
	}
	
	public ArrayList<DebrisData> getDebrisDataList()
	{
		return debrisDataList;
	}

	@Override
	public boolean epsilonEquals(HumanoidBehaviorDebrisPacket other, double epsilon) 
	{
		boolean ret = debrisDataList.size() == other.getDebrisDataList().size();
		
		for (int i = 0; i < debrisDataList.size(); i++)
		{
		   ret &= this.getDebrisDataList().get(i).epsilonEquals(other.getDebrisDataList().get(i), epsilon);
		}
		
		return ret;
	}

}
