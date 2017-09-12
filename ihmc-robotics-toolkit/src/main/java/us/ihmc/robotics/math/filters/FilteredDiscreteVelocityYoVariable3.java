package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
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
public class FilteredDiscreteVelocityYoVariable3 extends YoDouble
{
	private final YoDouble time;
	private final YoDouble position;
	
	private final YoDouble timeSinceLastPosChange;
	private final YoDouble lastPosChangeTimeInterval;
	
	private final TimestampedVelocityYoVariable finiteDifferenceVelocity;
	private final TimestampedVelocityYoVariable finiteDifferenceAccel;
	private final YoDouble velocityIfEncoderTicksNowConstantAccel;

	private boolean updateHasBeenCalled;
	private final YoBoolean assumeConstantAccel;
	
	private final double alpha;
	private final YoDouble alphaVariable;

	public FilteredDiscreteVelocityYoVariable3(String name, String description, double alpha, YoDouble time, YoVariableRegistry registry)
	{
		super(name, description, registry);
		
		this.time = time;
		this.position = null;

		timeSinceLastPosChange = new YoDouble(name + "_timeSinceLastTick", registry);
		lastPosChangeTimeInterval = new YoDouble(name + "_lastUpdateTimeInterval", registry);
		
		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		velocityIfEncoderTicksNowConstantAccel = new YoDouble(name + "_velocityIfEncoderTicksNowConstantAccel", registry);
		
		assumeConstantAccel = new YoBoolean(name + "_assumeConstAccel", registry);
		
		this.alpha = alpha;
		this.alphaVariable = null;
		
		reset();
	}

	public FilteredDiscreteVelocityYoVariable3(String name, String description, double alpha, YoDouble positionVariable, YoDouble time,
			YoVariableRegistry registry)
	{
		super(name, description, registry);

		this.time = time;
		this.position = positionVariable;

		timeSinceLastPosChange = new YoDouble(name + "_timeSinceLastTick", registry);
		lastPosChangeTimeInterval = new YoDouble(name + "_lastUpdateTimeInterval", registry);
		
		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		velocityIfEncoderTicksNowConstantAccel = new YoDouble(name + "_velocityIfEncoderTicksNowConstantAccel", registry);
		
		assumeConstantAccel = new YoBoolean(name + "_assumeConstAccel", registry);
		
		this.alpha = alpha;
		this.alphaVariable = null;
		
		reset();
	}

	public FilteredDiscreteVelocityYoVariable3(String name, String description, YoDouble alphaVariable, YoDouble positionVariable,
			YoDouble time, YoVariableRegistry registry)
	{
		super(name, description, registry);

		this.time = time;
		this.position = positionVariable;

		timeSinceLastPosChange = new YoDouble(name + "_timeSinceLastTick", registry);
		lastPosChangeTimeInterval = new YoDouble(name + "_lastUpdateTimeInterval", registry);
		
		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		velocityIfEncoderTicksNowConstantAccel = new YoDouble(name + "_velocityIfEncoderTicksNowConstantAccel", registry);
		
		assumeConstantAccel = new YoBoolean(name + "_assumeConstAccel", registry);
		
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
