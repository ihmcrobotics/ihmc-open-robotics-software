package us.ihmc.acsell.springs;

public class StepprLeftHipXSpringProperties implements HystereticSpringProperties {

	public StepprLeftHipXSpringProperties() {

	}
	
	@Override
	public double getLoadingSpringConstant() {
		return 270;
	}
	
	@Override
	public double getLinearSpringConstant() {
		return 355;
	}
	
	@Override
	public double getUnloadingSpringConstant() {
		return 350;
	}

	@Override
	public double getLoadingRestLength() {
		return 0.03;
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return -0.03;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return -0.05;
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
