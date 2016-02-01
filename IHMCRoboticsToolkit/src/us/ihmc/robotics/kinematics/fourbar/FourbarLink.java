/*
 * This class is based on interface and control code for STEPPR and WANDERER, 
 * two energy-efficient humanoid bipeds developed by the High Consequence Automation and Robotics Group at Sandia National Laboratories.
 *
 */
package us.ihmc.robotics.kinematics.fourbar;


public class FourbarLink {
	
	private double L;
	private double m;
	private double CoM_x;
	private double CoM_y;
	public FourbarLink(double length)
	{
		this.L = length;
		this.m = 0;
		this.CoM_x = 0;
		this.CoM_y = 0;
	}
	public FourbarLink(double length,double mass,double CoM_x,double CoM_y)
	{
		this.L = length;
		this.m = mass;
		this.CoM_x = CoM_x;
		this.CoM_y = CoM_y;
	}
	public FourbarLink(double length,double mass,double[] CoM)
	{
		this.L = length;
		this.m = mass;
		this.CoM_x = CoM[0];
		this.CoM_y = CoM[1];
	}
	
	public double getLength()
	{
		return L;
	}
	
	public double getMass()
	{
		return m;
	}
	
	public double getCoMx()
	{
		return CoM_x;
	}
	
	public double getCoMy()
	{
		return CoM_y;
	}
}
