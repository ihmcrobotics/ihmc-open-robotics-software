package us.ihmc.valkyrie.torquespeedcurve;

import us.ihmc.avatar.drcRobot.RobotTarget;

public class ValkyrieWalkingParameterValues {
	public double defaultSwingTime;
	public double defaultTransferTime;
	public double defaultInitialTransferTime;
	
	public ValkyrieWalkingParameterValues() {
		this(RobotTarget.REAL_ROBOT);
	}
	
	public ValkyrieWalkingParameterValues(RobotTarget target) {
		defaultSwingTime = (target == RobotTarget.REAL_ROBOT ? 1.20 : 0.60);
		defaultTransferTime = (target == RobotTarget.REAL_ROBOT ? 1.00 : 0.25);
		defaultInitialTransferTime = (target == RobotTarget.REAL_ROBOT ? 2.0 : 1.0);
	}

	public String toString() {
		return String.format("Default swing time: %f\nDefault transfer time: %f\nDefault initial transfer time: %f\n",
				defaultSwingTime, defaultTransferTime, defaultInitialTransferTime);
	}
}