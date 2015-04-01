package us.ihmc.acsell.springs;

public class StepprRightHipXSpringProperties implements HystereticSpringProperties {

	public StepprRightHipXSpringProperties() {

	}
	
//	@Override
//	public double getLoadingSpringConstant() {
//		return 330;
//	}
//	
//	@Override
//	public double getLinearSpringConstant() {
//		return 295;
//	}
//	
//	@Override
//	public double getUnloadingSpringConstant() {
//		return 260;
//	}
	
	@Override
	public double getLoadingSpringConstant() {
		return 300;
	}
	
	@Override
	public double getLinearSpringConstant() {
		return 280;
	}
	
	@Override
	public double getUnloadingSpringConstant() {
		return 300; //290
	}

//	@Override
//	public double getLoadingRestLength() {
//		return -0.02;
//	}
//	
//	@Override
//	public double getLinearSpringRestLength() {
//		return -0.028;
//	}
//	
//	@Override
//	public double getUnloadingRestLength() {
//		return -0.037;
//	}

	@Override
	public double getLoadingRestLength() {
		return -0.02;
	}
	
	@Override
	public double getLinearSpringRestLength() {
		return 0.0;
	}
	
	@Override
	public double getUnloadingRestLength() {
		return -0.06;
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
