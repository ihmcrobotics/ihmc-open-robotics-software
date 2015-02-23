package us.ihmc.wholeBodyController;

public enum DRCHandType {
	NONE(false), IROBOT(false), HOOK(false), ROBOTIQ(false), VALKYRIE(false);
	
	private final boolean simulateHand;
	
	private DRCHandType(boolean simulateHand)
	{
	   this.simulateHand = simulateHand;
	}
	
	public boolean isHandSimulated()
	{
	   return simulateHand;
	}
}
