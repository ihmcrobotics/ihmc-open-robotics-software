package us.ihmc.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class WalkToGoalBehaviorPacket extends Packet<WalkToGoalBehaviorPacket> {
   public static enum WalkToGoalAction{
      FIND_PATH, EXECUTE, EXECUTE_UNKNOWN, STOP
   };

	public WalkToGoalAction action;
	public double xGoal;
	public double yGoal;
	public double thetaGoal;

	public RobotSide goalSide;
	
	
	public WalkToGoalBehaviorPacket(){
		// for serialization
	}
	
	public WalkToGoalBehaviorPacket(WalkToGoalAction action){
		this.action = action;
	}
	
	public WalkToGoalBehaviorPacket(double xGoal, double yGoal, double thetaGoal, RobotSide goalSide){
		this.action = WalkToGoalAction.FIND_PATH;
		this.xGoal = xGoal;
		this.yGoal = yGoal;
		this.thetaGoal = thetaGoal;
		this.goalSide = goalSide;
	}
	
	public double[] getGoalPosition(){
		return new double[]{xGoal, yGoal, thetaGoal};
	}
	
	public RobotSide getGoalSide() {
		return goalSide;
	}
	
	@Override
	public boolean epsilonEquals(WalkToGoalBehaviorPacket other, double epsilon) {
		boolean ret = true;
		double[] thisData = this.getGoalPosition();
		double[] otherData = other.getGoalPosition();
		for (int i = 0; (i < thisData.length && ret); i++){
			ret = Math.abs(thisData[i] - otherData[i]) < epsilon;
		}
		ret &= this.goalSide == other.goalSide;
		return ret;
	}

}
