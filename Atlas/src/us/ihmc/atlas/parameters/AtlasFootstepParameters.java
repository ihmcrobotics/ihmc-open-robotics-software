package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.humanoidBehaviors.planning.FootstepParameters;
import us.ihmc.humanoidBehaviors.planning.FootstepPlanState;
import us.ihmc.utilities.robotSide.RobotSide;

public class AtlasFootstepParameters extends FootstepParameters{
	private static final double goalRadius = 0.25;
	private final double baseOffset = 0.3;
	public double maxStepUp = 0.20;
	public double minStepDown = -0.17;
	public double maxStepDistance = 0.6;
	
	private double yawCostGain = 1;
	
	public AtlasFootstepParameters(){
		initialize();
	}
	
	private void initialize()
	{
		setOffsets();
		footWidth = .14;
		footLength = .26;
	}
	
	private void setOffsets(){
		List<FootstepOffset> offsets = new ArrayList<FootstepOffset>();
      double[] xs = new double[]{0.0 , 0.0 , 0.0 , 0.6 , -0.3};
      double[] ys = new double[]{0.16 , 0.25 , 0.6 , 0.25 , 0.25};
      double[] thetas = new double[]{0.0 , 0.0 , 0.0 , 0.0 , 0.0};
      
      double[] thetaForwards = new double[]{-(Math.PI/16) , 0 , Math.PI/16 , Math.PI/8 , Math.PI/4 , 3*Math.PI/8};
      double[] thetaBackwards = new double[]{-(Math.PI/16) , 0 , Math.PI/16};

      for (int i = 0; i < xs.length; i++){
         offsets.add(new FootstepOffset(xs[i],ys[i], thetas[i]));
      }
      for (double theta : thetaForwards){
    	  offsets.add(new FootstepOffset(.3*Math.cos(theta), .25 + .3*Math.sin(theta), theta));
      }
      for (double theta : thetaBackwards){
    	  offsets.add(new FootstepOffset(-.3*Math.cos(theta), .25 + -.3*Math.sin(theta), theta));
      }
      
      offsetList = offsets;
	}

	public FootstepPlanState generateFromOffset(FootstepPlanState currentState, FootstepOffset offset){
		FootstepPlanState generated = new FootstepPlanState(currentState);
		double sign;
		if (currentState.side == RobotSide.LEFT){
			generated.side = RobotSide.RIGHT;
			sign  = -1;
		}else{
			generated.side = RobotSide.LEFT;
			sign = 1;
		}

		double xForward = (offset.dx) * Math.cos(generated.theta) - (sign * offset.dy) * Math.sin(generated.theta);
		generated.x += xForward;
		generated.y += (sign * offset.dy) * Math.cos(generated.theta) + (offset.dx) * Math.sin(generated.theta);
		generated.theta += sign * offset.dtheta;
		generated.theta = ((generated.theta +Math.PI) % (2 * Math.PI)) - Math.PI; 
		
		generated.costToState = currentState.costToState + 1;
		generated.costToState += Math.sqrt(Math.pow(offset.dx, 2)+ Math.pow(offset.dy,  2)) / maxStepDistance;
		generated.costToState -= xForward / maxStepDistance;
		generated.costToState += yawCostGain * offset.dtheta;
		
		return generated;
	}

	public boolean withinReach(FootstepPlanState currentState,
			FootstepPlanState goalState) {
		if (currentState.side == goalState.side){
			return false;
		}
		double xdiffworld = goalState.x - currentState.x;
		double ydiffworld = goalState.y - currentState.y;
		double theta = currentState.theta;
		double xdiff = xdiffworld * Math.cos(theta) + ydiffworld * Math.sin(theta);
		double ydiff = ydiffworld * Math.cos(theta) - xdiffworld * Math.sin(theta);
		if (xdiff >.3 || xdiff < -.3){
			return false;
		}
		if (currentState.side == RobotSide.LEFT){
			ydiff *= -1;
		}
		if (ydiff >0.5 || ydiff < 0.2){
			return false;
		}
		double thetadiff = (goalState.theta - currentState.theta + Math.PI)%(2*Math.PI) - Math.PI;
		if (thetadiff> Math.PI/16 || thetadiff < -Math.PI/16){
			return false;
		}
		return true;
	}
	
	public double getMaxStepUp(){
		return this.maxStepUp;
	}
	public double getMinStepDown(){
		return minStepDown;
	}
	public double getMaxStepDistance() {
		return maxStepDistance;	}
}
