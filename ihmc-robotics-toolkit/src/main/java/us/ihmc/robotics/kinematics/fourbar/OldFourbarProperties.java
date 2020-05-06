/*
 * This class is based on interface and control code for STEPPR and WANDERER, 
 * two energy-efficient humanoid bipeds developed by the High Consequence Automation and Robotics Group at Sandia National Laboratories.
 *
 */
package us.ihmc.robotics.kinematics.fourbar;


public interface OldFourbarProperties
{
	public OldFourbarLink getGroundLink();
	public OldFourbarLink getInputLink();
	public OldFourbarLink getFloatingLink();
	public OldFourbarLink getOutputLink();
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
