package us.ihmc.acsell.fourbar;

public interface FourbarProperties
{
	public FourbarLink getGroundLink();
	public FourbarLink getInputLink();
	public FourbarLink getFloatingLink();
	public FourbarLink getOutputLink();
	public boolean isElbowDown();
   public abstract double getRightLinkageBeta0();
   public abstract double getLeftLinkageBeta0();
}
