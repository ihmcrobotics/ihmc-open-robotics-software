package us.ihmc.acsell.springs;

public interface SpringProperties {
	

	public double getLinearSpringConstant();	
	public double getLinearSpringRestLength();
	public double getDirectionallity(); //+1.0 Positive is Tension OR -1.0 Negative is Tension
	public boolean isCompression();	
	public boolean isExtension();
	
}
