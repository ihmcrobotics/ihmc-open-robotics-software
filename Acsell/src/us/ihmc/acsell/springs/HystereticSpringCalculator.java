package us.ihmc.acsell.springs;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

public class HystereticSpringCalculator implements SpringCalculator {

	private final HystereticSpringProperties springProperties;
	private double F;
	private double x;
	private double last_x;
	private double k_L;
	private double x0_L;
	private double k_U;
	private double x0_U;
	private final DoubleYoVariable s;
	private final AlphaFilteredYoVariable s_filt;
	
	public HystereticSpringCalculator(HystereticSpringProperties springProperties, String name, YoVariableRegistry registry)
	{
		this.springProperties = springProperties;
		this.k_L = springProperties.getLoadingSpringConstant();
		this.x0_L = springProperties.getLoadingRestLength();
		this.k_U = springProperties.getUnloadingSpringConstant();
		this.x0_U = springProperties.getUnloadingRestLength();
		
		s = new DoubleYoVariable(name + "SpringLoadingState", registry);
		s_filt = new AlphaFilteredYoVariable(name + "FilteredSpringLoadingState", registry, 0.95, s);
		
	}
	
	@Override
	public void update(double currentLength) {
		x = currentLength*springProperties.getDirectionallity();
		s.set(Math.signum(last_x-x)*0.5+0.5);
		s_filt.update();
		F = isSpringEngaged() ? -k_L*(x-x0_L)*s_filt.getDoubleValue() -k_U*(x-x0_U)*(1-s_filt.getDoubleValue()) : 0;
		
		last_x = x;
	}

	@Override
	public double getSpringForce() {
		return F*springProperties.getDirectionallity();
	}

	@Override
	public boolean isSpringEngaged() {
		if (isLoading())
		{
			if(x<x0_L && springProperties.isCompression())
				return true;
			if(x>x0_L && springProperties.isExtension())
				return true;
		} else {
			if(x<x0_U && springProperties.isCompression())
				return true;
			if(x>x0_U && springProperties.isExtension())
				return true;
		}			
		return false;
	}
	
	public boolean isLoading() //TODO: This only works with compression springs...
	{
		return s_filt.getDoubleValue() > 0.5;
	}
	
	public boolean isUnloading()
	{
		return !isLoading();
	}

}
