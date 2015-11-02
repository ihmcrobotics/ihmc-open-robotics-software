package us.ihmc.steppr.hardware.configuration;

import us.ihmc.acsell.fourbar.FourbarLink;
import us.ihmc.acsell.fourbar.FourbarProperties;

public class StepprFourbarProperties implements FourbarProperties {

	private static final double LEFT_LINKAGE_BETA0 = 2.40;
	private static final double RIGHT_LINKAGE_BETA0 = 2.56;
	private final double L1 = 3.57*0.0254;
	private final double L2 = 5.2*0.0254;
	private final double L3 = 5.3*0.0254;
	private final double L4 = 5.25*0.0254;
	private final FourbarLink Link1;
	private final FourbarLink Link2;
	private final FourbarLink Link3;
	private final FourbarLink Link4;
	
	public StepprFourbarProperties()
	{
		this.Link1 = new FourbarLink(L1);
		this.Link2 = new FourbarLink(L2);
		this.Link3 = new FourbarLink(L3);
		this.Link4 = new FourbarLink(L4);
	}
	
	@Override
   public double getLeftLinkageBeta0()
	{
	   return LEFT_LINKAGE_BETA0;
	}
	
	@Override
   public double getRightLinkageBeta0()
	{
	   return RIGHT_LINKAGE_BETA0;
	}
	
	@Override
	public FourbarLink getGroundLink() {
		return Link1;
	}

	@Override
	public FourbarLink getInputLink() {
		return Link2;
	}

	@Override
	public FourbarLink getFloatingLink() {
		return Link3;
	}

	@Override
	public FourbarLink getOutputLink() {
		return Link4;
	}

	@Override
	public boolean isElbowDown() {
		return true;
	}

}
