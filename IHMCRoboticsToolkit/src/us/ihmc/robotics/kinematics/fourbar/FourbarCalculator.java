/*
 * This class is based on interface and control code for STEPPR and WANDERER, 
 * two energy-efficient humanoid bipeds developed by the High Consequence Automation and Robotics Group at Sandia National Laboratories.
 *
 */
package us.ihmc.robotics.kinematics.fourbar;

 public class FourbarCalculator {
	private final FourbarProperties Fourbar;	
	private final double L1;
	private final double L2;
	private final double L3;
	private final double L4;	
	
	private double phi;
	private double x;
	private double y;
	private double D;
	private double Dsqrd;
	private double beta; //External angle from GroundLink to OutputLink
	private double alpha;
	private double a1;
	private double a2;
	private double da1db;
	private double da2db;
	private double N;
		
	public FourbarCalculator(FourbarProperties fourbar)
	{
		this.Fourbar = fourbar;
        L1 = this.Fourbar.getGroundLink().getLength();
        L2 = this.Fourbar.getInputLink().getLength();
        L3 = this.Fourbar.getFloatingLink().getLength();
        L4 = this.Fourbar.getOutputLink().getLength();
	}
	
	private double getElbowSign()
	{
		return Fourbar.isElbowDown() ? -1.0 : 1.0;		
	}
		
	public void updateFourbarKinematicEquationsFromOutputAngle()
	{			
		phi = Math.PI - beta; //Internal angle from GroundLink to OutputLink
		x =  L1 - L4*Math.cos(phi); //Distance from L1-L2 joint to L4-L3 joint as measured parallel to L1
		y = L4*Math.sin(phi); //Distance from L1-L2 joint to L4-L3 joint as measured perpendicular to L1
		Dsqrd = x*x+y*y; //Distance squared from L1-L2 joint to L4-L3
		D = Math.sqrt(Dsqrd); //Distance from L1-L2 joint to L4-L3
		a1 = Math.atan2(y, x); //Angle from L1 to the L4-L3 joint
		a2 = getElbowSign()*Math.acos((L2*L2 + Dsqrd - L3*L3)/(2*L2*D)); //Angle from L2 to the L4-L3 joint
		da1db = 1-L1*x/Dsqrd; //Derivative of a1 wrt beta
		da2db = L1*y*(D-L2*Math.cos(a2))/(L2*Dsqrd*Math.sin(a2)); //Derivative of a2 wrt beta
		N = da1db+da2db;
		alpha = a1+a2;
	}
	
	public double getFourbarRatio()
	{
		return N;
	}
	
	public double getInputAngle()
	{
		return alpha;
	}
	
	public void setOutputAngle(double beta)
	{
		this.beta = beta;
	}
	
	//TODO: updateDynamicEquations
	
	
}
