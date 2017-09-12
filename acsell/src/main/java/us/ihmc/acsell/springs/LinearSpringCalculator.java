package us.ihmc.acsell.springs;

public class LinearSpringCalculator implements SpringCalculator {

	private final SpringProperties springProperties;
	private double F;
	private double x;
	private double k;
	private double x0;
	
	public LinearSpringCalculator(SpringProperties springProperties)
	{
		this.springProperties = springProperties;
		this.k = springProperties.getLinearSpringConstant();
		this.x0 = springProperties.getLinearSpringRestLength();
	}
	
	@Override
	public void update(double currentLength) {
		x = currentLength*springProperties.getDirectionallity();
		F = isSpringEngaged() ? -k*(x-x0) : 0;
	}

	@Override
	public double getSpringForce() {
		return F*springProperties.getDirectionallity();
	}

	@Override
	public boolean isSpringEngaged() {
		if(x<x0 && springProperties.isCompression())
			return true;
		if(x>x0 && springProperties.isExtension())
			return true;
		return false;
	}	

}
