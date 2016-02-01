package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.TimestampedVelocityYoVariable;

/**
 * @author ChrisSchmidtWetekam
 *         <p>
 *         If encoder changes counts
 *         
 *         		Compute velocity  using finite differences
 *         
 *         Else
 *         
 *         		Do not assume constant velocity.
 *         		Instead, assume constant acceleration (computed using finite differences),
 *         		and update velocity accordingly
 *         
 *         Low pass filter the whole thing:
 *         </p>
 *
 * <pre>
 *            vel_{n} = alpha * vel{n-1} + (1 - alpha) * (pos_{n} - pos_{n-1})
 * </pre>
 *
 */
public class FilteredDiscreteVelocityYoVariable3 extends DoubleYoVariable
{
	private final DoubleYoVariable time;
	private final DoubleYoVariable position;
	
	private final DoubleYoVariable timeSinceLastPosChange;
	private final DoubleYoVariable lastPosChangeTimeInterval;
	
	private final TimestampedVelocityYoVariable finiteDifferenceVelocity;
	private final TimestampedVelocityYoVariable finiteDifferenceAccel;
	private final DoubleYoVariable velocityIfEncoderTicksNowConstantAccel;

	private boolean updateHasBeenCalled;
	private final BooleanYoVariable assumeConstantAccel;
	
	private final double alpha;
	private final DoubleYoVariable alphaVariable;

	public FilteredDiscreteVelocityYoVariable3(String name, String description, double alpha, DoubleYoVariable time, YoVariableRegistry registry)
	{
		super(name, description, registry);
		
		this.time = time;
		this.position = null;

		timeSinceLastPosChange = new DoubleYoVariable(name + "_timeSinceLastTick", registry);
		lastPosChangeTimeInterval = new DoubleYoVariable(name + "_lastUpdateTimeInterval", registry);
		
		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		velocityIfEncoderTicksNowConstantAccel = new DoubleYoVariable(name + "_velocityIfEncoderTicksNowConstantAccel", registry);
		
		assumeConstantAccel = new BooleanYoVariable(name + "_assumeConstAccel", registry);
		
		this.alpha = alpha;
		this.alphaVariable = null;
		
		reset();
	}

	public FilteredDiscreteVelocityYoVariable3(String name, String description, double alpha, DoubleYoVariable positionVariable, DoubleYoVariable time,
			YoVariableRegistry registry)
	{
		super(name, description, registry);

		this.time = time;
		this.position = positionVariable;

		timeSinceLastPosChange = new DoubleYoVariable(name + "_timeSinceLastTick", registry);
		lastPosChangeTimeInterval = new DoubleYoVariable(name + "_lastUpdateTimeInterval", registry);
		
		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		velocityIfEncoderTicksNowConstantAccel = new DoubleYoVariable(name + "_velocityIfEncoderTicksNowConstantAccel", registry);
		
		assumeConstantAccel = new BooleanYoVariable(name + "_assumeConstAccel", registry);
		
		this.alpha = alpha;
		this.alphaVariable = null;
		
		reset();
	}

	public FilteredDiscreteVelocityYoVariable3(String name, String description, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable,
			DoubleYoVariable time, YoVariableRegistry registry)
	{
		super(name, description, registry);

		this.time = time;
		this.position = positionVariable;

		timeSinceLastPosChange = new DoubleYoVariable(name + "_timeSinceLastTick", registry);
		lastPosChangeTimeInterval = new DoubleYoVariable(name + "_lastUpdateTimeInterval", registry);
		
		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		velocityIfEncoderTicksNowConstantAccel = new DoubleYoVariable(name + "_velocityIfEncoderTicksNowConstantAccel", registry);
		
		assumeConstantAccel = new BooleanYoVariable(name + "_assumeConstAccel", registry);
		
		this.alphaVariable = alphaVariable;
		this.alpha = 0.0;
		
		reset();
	}

	public void reset()
	{
		updateHasBeenCalled = false;
		assumeConstantAccel.set(true);
	}

	public void update()
	{
		if (position == null)
		{
			throw new NullPointerException("YoFilteredVelocityVariable must be constructed with a non null "
					+ "position variable to call update(), otherwise use update(double)");
		}

		update(position.getDoubleValue());
	}

	public void update(double currentPosition)
	{
		if (!updateHasBeenCalled)
		{
			updateHasBeenCalled = true;
		}
		
		timeSinceLastPosChange.set( time.getDoubleValue() - finiteDifferenceVelocity.getPreviousTimestamp() );		

		double previousPositon = finiteDifferenceVelocity.getPreviousPosition();
		

		if ( currentPosition != previousPositon )
		{
			this.finiteDifferenceVelocity.update();
			this.finiteDifferenceAccel.update();
			
			lastPosChangeTimeInterval.set( timeSinceLastPosChange.getDoubleValue() );
		}

		else
		{
			velocityIfEncoderTicksNowConstantAccel.set( finiteDifferenceVelocity.getDoubleValue() + finiteDifferenceAccel.getDoubleValue() * timeSinceLastPosChange.getDoubleValue());
		}

		
		/************** SET VALUE *************************/
		if ( assumeConstantAccel.getBooleanValue() )
		{
			this.set( alphaFilter( velocityIfEncoderTicksNowConstantAccel.getDoubleValue() ) );
		}
		else
		{
			this.set( alphaFilter( finiteDifferenceVelocity.getDoubleValue() ));
		}
	}

	private double alphaFilter(double currentValue)
	{
		double previousFilteredValue = this.getDoubleValue();

		double a = this.alpha;
		if (alphaVariable != null)
		{
			a = alphaVariable.getDoubleValue();
		}

		double ret = a * previousFilteredValue + (1.0 - a) * currentValue;

		return ret;
	}

}
