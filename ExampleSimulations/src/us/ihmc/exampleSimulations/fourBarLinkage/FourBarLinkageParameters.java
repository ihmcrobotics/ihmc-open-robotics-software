package us.ihmc.exampleSimulations.fourBarLinkage;

public class FourBarLinkageParameters {
	
	/**
	 * Each link is given a number 1-4, with the root joint being 1,
	 * it's child joint 2, and so on. Joint 3 is connected to joint 1
	 * via an external force point, not a pin joint
	 */

	public double linkageLength_1, linkageLength_2, linkageLength_3, linkageLength_4;
	public double damping_1, damping_2, damping_3, damping_4;
	public double mass_1, mass_2, mass_3, mass_4;
	public double radius_1, radius_2, radius_3, radius_4;
	public double angle_1, angle_2, angle_3;
	
	public double k_Constraint, kd_Constraint;	
}
