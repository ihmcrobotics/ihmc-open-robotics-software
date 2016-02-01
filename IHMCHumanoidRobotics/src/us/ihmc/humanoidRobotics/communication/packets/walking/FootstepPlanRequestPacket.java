package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class FootstepPlanRequestPacket extends Packet<FootstepPlanRequestPacket> {
	
	public enum RequestType{
		START_SEARCH, STOP_SEARCH, UPDATE_START
	};

	public FootstepData startFootstep;
	public double thetaStart;
	public double maxSuboptimality = 1;
	
	public ArrayList<FootstepData> goals = new ArrayList<FootstepData>();

	public RequestType requestType;
	
	public FootstepPlanRequestPacket(Random random)
	{
	   startFootstep = new FootstepData(random);
	   thetaStart = random.nextDouble();
	   maxSuboptimality = random.nextDouble();
	}

	public FootstepPlanRequestPacket()
	{
		// for serialization.
	}
	
	public FootstepPlanRequestPacket(RequestType requestType, FootstepData startFootstep, double thetaStart, ArrayList<FootstepData> goals)
	{
		this.requestType = requestType;
		this.startFootstep = startFootstep;
		this.thetaStart = thetaStart;
		this.goals = goals;
	}

	public FootstepPlanRequestPacket(RequestType requestType, FootstepData startFootstep, double thetaStart, ArrayList<FootstepData> goals, double maxSuboptimality)
	{
		this.requestType = requestType;
		this.startFootstep = startFootstep;
		this.thetaStart = thetaStart;
		this.goals = goals;
		this.maxSuboptimality = maxSuboptimality;
	}

	@Override
	public boolean epsilonEquals(FootstepPlanRequestPacket other, double epsilon) {
		if (this.requestType != other.requestType) return false;
		if (Math.abs(this.thetaStart - other.thetaStart) > epsilon) return false;
		if (Math.abs(this.maxSuboptimality - other.maxSuboptimality) > epsilon) return false;
		if (!this.startFootstep.epsilonEquals(other.startFootstep, epsilon)) return false;
		if (this.goals.size() != other.goals.size()) return false;
		for (int i = 0; i < goals.size(); i++){
			if (!goals.get(i).epsilonEquals(other.goals.get(i), epsilon)) return false;
		}
		return true;
	}
}
