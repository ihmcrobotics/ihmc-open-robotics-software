package us.ihmc.valkyrie.torquespeedcurve;

public class ValkyrieWalkingParameterValues {
	public double defaultSwingTime;
	public double defaultTransferTime;
	public double defaultInitialTransferTime;
	
	public ValkyrieWalkingParameterValues() {
		defaultSwingTime = 0.60;
		defaultTransferTime = 0.25;
		defaultInitialTransferTime = 1.0;
	}

	public String toString() {
		return String.format("Default swing time: %f\nDefault transfer time: %f\nDefault initial transfer time: %f\n",
				defaultSwingTime, defaultTransferTime, defaultInitialTransferTime);
	}
}
