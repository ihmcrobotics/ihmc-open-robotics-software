/*
 * This class is based on interface and control code for STEPPR and WANDERER, 
 * two energy-efficient humanoid bipeds developed by the High Consequence Automation and Robotics Group at Sandia National Laboratories.
 *
 */
package us.ihmc.robotics.kinematics.fourbar;


public interface FourbarProperties
{
	public FourbarLink getGroundLink();
	public FourbarLink getInputLink();
	public FourbarLink getFloatingLink();
	public FourbarLink getOutputLink();
	public boolean isElbowDown();
	
	/**
	 * Acsell specific, feel free to ignore
	 */
   public abstract double getRightLinkageBeta0();
   /**
    * Acsell specific, feel free to ignore
    */
   public abstract double getLeftLinkageBeta0();
}
