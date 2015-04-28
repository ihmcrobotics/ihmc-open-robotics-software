package us.ihmc.wanderer.hardware.configuration;

import us.ihmc.acsell.springs.HystereticSpringProperties;

public class WandererRightHipXSpringProperties implements HystereticSpringProperties {

	public WandererRightHipXSpringProperties() {
	   throw new RuntimeException("TODO: Change values for wanderer and remove this exception");
	}
	
	@Override
	public double getLoadingSpringConstant() {
		return 400;
	}
	
	@Override
	public double getLinearSpringConstant() {
		return 280;
	}
	
	@Override
	public double getUnloadingSpringConstant() {
		return 380; //290
	}

	@Override
	public double getLoadingRestLength() {
		return 0.03;
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return 0.0;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return -0.01;
	}
	
	@Override
	public boolean isCompression() {
		return true;
	}

	@Override
	public boolean isExtension() {
		return false;
	}

	@Override
	public double getDirectionallity() {
		return -1.0; //Right Leg Compresses at Positive Angles
	}

}
