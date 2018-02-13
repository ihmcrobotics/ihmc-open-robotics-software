package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import us.ihmc.communication.packets.Packet;

public class FootstepPlanRequestPacket extends Packet<FootstepPlanRequestPacket> {
	
	public enum RequestType{
		START_SEARCH, STOP_SEARCH, UPDATE_START
	};

	public FootstepDataMessage startFootstep;
	public double thetaStart;
	public double maxSuboptimality = 1;
	
	public ArrayList<FootstepDataMessage> goals = new ArrayList<FootstepDataMessage>();

	public RequestType requestType;

	public FootstepPlanRequestPacket()
	{
		// for serialization.
	}
	
	public FootstepPlanRequestPacket(RequestType requestType, FootstepDataMessage startFootstep, double thetaStart, ArrayList<FootstepDataMessage> goals)
	{
		this.requestType = requestType;
		this.startFootstep = startFootstep;
		this.thetaStart = thetaStart;
		this.goals = goals;
	}

	public FootstepPlanRequestPacket(RequestType requestType, FootstepDataMessage startFootstep, double thetaStart, ArrayList<FootstepDataMessage> goals, double maxSuboptimality)
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
