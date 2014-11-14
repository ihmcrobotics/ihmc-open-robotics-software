package us.ihmc.atlas.parameters;


import us.ihmc.ihmcPerception.footstepPlanner.FootstepParameters;

import java.util.ArrayList;
import java.util.List;

public class AtlasFootstepParameters extends FootstepParameters {
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

	public double getMaxStepUp(){
		return this.maxStepUp;
	}
	public double getMinStepDown(){
		return minStepDown;
	}
	public double getMaxStepDistance() {
		return maxStepDistance;	}
}
