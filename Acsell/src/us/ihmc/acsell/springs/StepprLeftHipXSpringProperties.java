package us.ihmc.acsell.springs;

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
		return 370;
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
		return -0.00;//0.03; //Both spacers 0.03, larger spacer 0.00, small spacer -0.05
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return 0.015;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return -0.03;//0.00 //Loading -0.03
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
