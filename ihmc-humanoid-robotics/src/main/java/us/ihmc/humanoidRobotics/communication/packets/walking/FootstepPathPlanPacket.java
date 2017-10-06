package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class FootstepPathPlanPacket extends Packet<FootstepPathPlanPacket> {
	
	public boolean goalsValid;
	public FootstepDataMessage start;
	public ArrayList<FootstepDataMessage> originalGoals = new ArrayList<FootstepDataMessage>();
	public ArrayList<FootstepDataMessage> pathPlan = new ArrayList<FootstepDataMessage>();
	public ArrayList<Boolean> footstepUnknown = new ArrayList<Boolean>();
	public double subOptimality;
	public double pathCost = Double.POSITIVE_INFINITY;
	
	public FootstepPathPlanPacket(Random random)
	{
	   goalsValid = random.nextBoolean();
	   start = new FootstepDataMessage(random);
	   int size = Math.abs(random.nextInt(1000));
	   for (int i = 0; i < size; i++)
      {
         originalGoals.add(new FootstepDataMessage(random));
      }
	   
	   size = Math.abs(random.nextInt(1000));
	   for (int i = 0; i < size; i++)
	   {
	      pathPlan.add(new FootstepDataMessage(random));
	   }
	   
	   size = Math.abs(random.nextInt(1000));
	   for (int i = 0; i < size; i++)
	   {
	      footstepUnknown.add(random.nextBoolean());
	   }
	   
	   subOptimality = random.nextDouble();
	   pathCost = random.nextDouble();
	   
	}
	
	public FootstepPathPlanPacket()
	{
		// empty constructor for serialization
	}
	
	public FootstepPathPlanPacket(boolean goalsValid, FootstepDataMessage start, ArrayList<FootstepDataMessage> originalGoals, ArrayList<FootstepDataMessage> ADStarPathPlan,
			ArrayList<Boolean> footstepUnknown, double subOptimality, double cost)
	{
		this.goalsValid = goalsValid;
		this.start = start;
		this.originalGoals = originalGoals;
		this.pathPlan = ADStarPathPlan;
		this.footstepUnknown = footstepUnknown;
		this.subOptimality = subOptimality;
		this.pathCost = cost;
	}

	@Override
	public boolean epsilonEquals(FootstepPathPlanPacket other, double epsilon) {
		if (goalsValid != other.goalsValid) return false;
		if (!start.epsilonEquals(other.start, epsilon)) return false;
		if (originalGoals.size() != other.originalGoals.size()) return false;
		if (pathPlan.size() != other.pathPlan.size()) return false;
		if (footstepUnknown.size() != other.footstepUnknown.size()) return false;
		if (Math.abs(subOptimality-other.subOptimality)>epsilon) return false;
		if (Math.abs(pathCost-other.pathCost)>epsilon) return false;
		for (int i = 0; i < originalGoals.size(); i++){
			if (!originalGoals.get(i).epsilonEquals(other.originalGoals.get(i), epsilon)) return false;
		}
		for (int i = 0; i < pathPlan.size(); i++){
			if (!pathPlan.get(i).epsilonEquals(other.pathPlan.get(i), epsilon)) return false;
		}
		for (int i = 0; i < footstepUnknown.size(); i++){
			if (footstepUnknown.get(i) != other.footstepUnknown.get(i)) return false;
		}
		
		return true;
	}

}
