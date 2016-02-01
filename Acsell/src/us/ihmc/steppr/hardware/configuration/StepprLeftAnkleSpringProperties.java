package us.ihmc.steppr.hardware.configuration;

import us.ihmc.acsell.springs.HystereticSpringProperties;

public class StepprLeftAnkleSpringProperties implements HystereticSpringProperties {

	public StepprLeftAnkleSpringProperties() {

	}
	
	@Override
	public double getLoadingSpringConstant() {
		return 0.0;
	}
	
	@Override
	public double getLinearSpringConstant() {
		return 400;
	}
	
	@Override
	public double getUnloadingSpringConstant() {
		return 0.0;
	}
		
	@Override
	public double getLoadingRestLength() {
		return 0.0;
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return -0.228;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return 0.0;
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
		return 1.0; //Ankle Compresses at Negative Angles
	}
}
