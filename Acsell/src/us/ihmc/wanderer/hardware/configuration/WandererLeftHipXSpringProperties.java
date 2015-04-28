package us.ihmc.wanderer.hardware.configuration;

import us.ihmc.acsell.springs.HystereticSpringProperties;

public class WandererLeftHipXSpringProperties implements HystereticSpringProperties {

	public WandererLeftHipXSpringProperties() {
	   throw new RuntimeException("TODO: Change values for wanderer and remove this exception");
	}
	
	@Override
	public double getLoadingSpringConstant() {
		return 330;
	}
	
	@Override
	public double getLinearSpringConstant() {
		return 385;
	}
	
	@Override
	public double getUnloadingSpringConstant() {
		return 300;
	}
	
	@Override
	public double getLoadingRestLength() {
		return 0.04;//0.02;
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return 0.015;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return 0.02;//0.00 //Loading -0.03
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
		return 1.0; //Left Leg Compresses at Negative Angles
	}
}
