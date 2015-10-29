package us.ihmc.acsell.springs;

public interface SpringCalculator {
	
	public void update(double currentLength);
	
	public double getSpringForce();
	public boolean isSpringEngaged();

}
