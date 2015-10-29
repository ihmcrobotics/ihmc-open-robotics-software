package us.ihmc.steppr.hardware.configuration;

import us.ihmc.acsell.springs.HystereticSpringProperties;

public class StepprLeftHipXSpringProperties implements HystereticSpringProperties {

	public StepprLeftHipXSpringProperties() {

	}
	
//	@Override
//	public double getLoadingSpringConstant() {
//		return 270;
//	}
//	
//	@Override
//	public double getLinearSpringConstant() {
//		return 280;
//	}
//	
//	@Override
//	public double getUnloadingSpringConstant() {
//		return 200; //290
//	}
	
	@Override
	public double getLoadingSpringConstant() {
		return 370;
	}
	
	@Override
	public double getLinearSpringConstant() {
		return 385;
	}
	
	@Override
	public double getUnloadingSpringConstant() {
		return 340;
	}
	
//	@Override
//	public double getLoadingRestLength() {
//		return 0.03;
//	}
//	
//	@Override
//	public double getLinearSpringRestLength() {
//		return -0.03;
//	}
//	
//	@Override
//	public double getUnloadingRestLength() {
//		return -0.05;
//	}
	
	@Override
	public double getLoadingRestLength() {
		return 0.08;//0.02;
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return 0.015;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return 0.06;//0.00 //Loading -0.03
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
