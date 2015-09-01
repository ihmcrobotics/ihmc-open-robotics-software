package us.ihmc.wanderer.hardware.configuration;

import us.ihmc.acsell.springs.HystereticSpringProperties;

public class WandererLeftHipXSpringProperties implements HystereticSpringProperties {

	public WandererLeftHipXSpringProperties() {
	   //TODO: Change values for wanderer
	}
	
	@Override
	public double getLoadingSpringConstant() {
		return 0;
	}
	
	@Override
	public double getLinearSpringConstant() {
		return 950*0.9;
	}
	
	@Override
	public double getUnloadingSpringConstant() {
		return 0;
	}
	
	@Override
	public double getLoadingRestLength() {
		return 0.0;//0.02;
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return 0.014;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return 0.00;//0.00 //Loading -0.03
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
