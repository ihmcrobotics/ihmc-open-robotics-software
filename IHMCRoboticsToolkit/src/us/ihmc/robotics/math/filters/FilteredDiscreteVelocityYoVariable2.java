package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.TimestampedVelocityYoVariable;

/**
 * @author jrebula
 *         <p>
 *         A YoFilteredVelocityVariable is a filtered velocity of a position.
 *         Either a YoVariable holding the position is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The YoFilteredVelocityVariable updates its val
 *         with the current velocity after a filter of
 *         </p>
 *
 * <pre>
 *            vel_{n} = alpha * vel{n-1} + (1 - alpha) * (pos_{n} - pos_{n-1})
 * </pre>
 *
 */
public class FilteredDiscreteVelocityYoVariable2 extends DoubleYoVariable
{
	private final double alpha;

	private final DoubleYoVariable time;

	private final DoubleYoVariable alphaVariable;
	private final DoubleYoVariable position;

	private final DoubleYoVariable lastPosChangeTimeInterval;
	private final EnumYoVariable<Direction> lastPosChangeDirection;
	
	private final TimestampedVelocityYoVariable finiteDifferenceVelocity;
	private final DoubleYoVariable unfilteredVelocity;

	private final TimestampedVelocityYoVariable finiteDifferenceAccel;

	private boolean updateHasBeenCalled;

	private final DoubleYoVariable timeSinceLastPosChange;

	private final DoubleYoVariable lastPositionIncrement;
	private final DoubleYoVariable positionPredicted;

	private final DoubleYoVariable velocityIfEncoderTicksNow;
	private final DoubleYoVariable velocityIfEncoderTicksNowConstantAccel;


	private final BooleanYoVariable useDecay;

	public FilteredDiscreteVelocityYoVariable2(String name, String description, double alpha, DoubleYoVariable time, YoVariableRegistry registry)
	{
		super(name, description, registry);

		this.alpha = alpha;
		this.alphaVariable = null;
		this.position = null;

		this.time = time;


		lastPosChangeTimeInterval = new DoubleYoVariable(name + "_lastUpdateTimeInterval", registry);
		lastPosChangeDirection = EnumYoVariable.create(name + "_lastUpdateDirection", Direction.class, registry);
		
		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		unfilteredVelocity = new DoubleYoVariable(name + "_unfiltered", registry);
		
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		
		timeSinceLastPosChange = new DoubleYoVariable(name + "_timeSinceLastTick", registry);

		lastPositionIncrement = new DoubleYoVariable(name + "_lastPositionIncrement", registry);
		positionPredicted = new DoubleYoVariable(name + "_positionPredicted", registry);

		velocityIfEncoderTicksNow = new DoubleYoVariable(name + "_velocityIfEncoderTicksNow", registry);
		velocityIfEncoderTicksNowConstantAccel = new DoubleYoVariable(name + "_velocityIfEncoderTicksNowConstantAccel", registry);
		
		useDecay = new BooleanYoVariable(name + "_useDecay", registry);
		
		reset();
	}

	public FilteredDiscreteVelocityYoVariable2(String name, String description, double alpha, DoubleYoVariable positionVariable, DoubleYoVariable time,
			YoVariableRegistry registry)
	{
		super(name, description, registry);

		this.alpha = alpha;
		this.position = positionVariable;

		this.alphaVariable = null;

		this.time = time;


		lastPosChangeTimeInterval = new DoubleYoVariable(name + "_lastUpdateTimeInterval", registry);
		lastPosChangeDirection = EnumYoVariable.create(name + "_lastUpdateDirection", Direction.class, registry);

		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		unfilteredVelocity = new DoubleYoVariable(name + "_unfiltered", registry);
		
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", finiteDifferenceVelocity, time, registry, 1e-20);
		
		timeSinceLastPosChange = new DoubleYoVariable(name + "_timeSinceLastTick", registry);

		lastPositionIncrement = new DoubleYoVariable(name + "_lastPositionIncrement", registry);
		positionPredicted = new DoubleYoVariable(name + "_positionPredicted", registry);

		velocityIfEncoderTicksNow = new DoubleYoVariable(name + "_velocityIfEncoderTicksNow", registry);
		velocityIfEncoderTicksNowConstantAccel = new DoubleYoVariable(name + "_velocityIfEncoderTicksNowConstantAccel", registry);

		useDecay = new BooleanYoVariable(name + "_useDecay", registry);
		
		reset();
	}

	public FilteredDiscreteVelocityYoVariable2(String name, String description, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable,
			DoubleYoVariable time, YoVariableRegistry registry)
	{
		super(name, description, registry);
		position = positionVariable;
		this.alphaVariable = alphaVariable;
		this.alpha = 0.0;

		this.time = time;


		lastPosChangeTimeInterval = new DoubleYoVariable(name + "_lastUpdateTimeInterval", registry);
		lastPosChangeDirection = EnumYoVariable.create(name + "_lastUpdateDirection", Direction.class, registry);

		finiteDifferenceVelocity = new TimestampedVelocityYoVariable(name + "_finiteDiff", "", position, time, registry, 1e-20);
		unfilteredVelocity = new DoubleYoVariable(name + "_unfiltered", registry);
		
		finiteDifferenceAccel = new TimestampedVelocityYoVariable(name + "_finiteDiffAccel", "", unfilteredVelocity, time, registry, 1e-20);

		timeSinceLastPosChange = new DoubleYoVariable(name + "_timeSinceLastTick", registry);

		lastPositionIncrement = new DoubleYoVariable(name + "_lastPositionIncrement", registry);
		positionPredicted = new DoubleYoVariable(name + "_positionPredicted", registry);

		velocityIfEncoderTicksNow = new DoubleYoVariable(name + "_velocityIfEncoderTicksNow", registry);
		velocityIfEncoderTicksNowConstantAccel = new DoubleYoVariable(name + "_velocityIfEncoderTicksNowConstantAccel", registry);

		useDecay = new BooleanYoVariable(name + "_useDecay", registry);
		
		reset();
	}

	public void reset()
	{
		updateHasBeenCalled = false;
		useDecay.set(true);
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
			lastPosChangeDirection.set(Direction.NONE);
		}
		
		timeSinceLastPosChange.set( time.getDoubleValue() - finiteDifferenceVelocity.getPreviousTimestamp() );		


		double previousPositon = finiteDifferenceVelocity.getPreviousPosition();
		
		velocityIfEncoderTicksNow.set( ( positionPredicted.getDoubleValue() - previousPositon ) / timeSinceLastPosChange.getDoubleValue() );

		
		if ( currentPosition != previousPositon )
		{
			this.finiteDifferenceVelocity.update();
			this.finiteDifferenceAccel.update();

			if ( determineIfDirectionChanged(currentPosition) )
			{
				unfilteredVelocity.set(0.0);
				positionPredicted.set( currentPosition - lastPositionIncrement.getDoubleValue());
			}
			else
			{
				unfilteredVelocity.set(finiteDifferenceVelocity.getDoubleValue());
				positionPredicted.set( currentPosition + lastPositionIncrement.getDoubleValue());
			}
			
			lastPosChangeTimeInterval.set( timeSinceLastPosChange.getDoubleValue() );
			lastPositionIncrement.set( currentPosition - previousPositon );
		}

		else
		{
			velocityIfEncoderTicksNowConstantAccel.set( finiteDifferenceVelocity.getDoubleValue() + finiteDifferenceAccel.getDoubleValue() * timeSinceLastPosChange.getDoubleValue());

			boolean velSlowedDownSincePosTakingTooLongToChange = timeSinceLastPosChange.getDoubleValue() > lastPosChangeTimeInterval.getDoubleValue();

			if( velSlowedDownSincePosTakingTooLongToChange && useDecay.getBooleanValue() )
			{
				this.unfilteredVelocity.set( velocityIfEncoderTicksNow.getDoubleValue() );
			}

		}

		
		/************** SET VALUE *************************/
		this.set( alphaFilter(velocityIfEncoderTicksNowConstantAccel.getDoubleValue()) );

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
	

	private Direction computeDirectionOfMotion(double currentPosition)
	{
		Direction directionOfMotion;

		if (currentPosition > finiteDifferenceVelocity.getPreviousPosition())
		{
			directionOfMotion = Direction.FORWARD;
		}
		else if (currentPosition < finiteDifferenceVelocity.getPreviousPosition())
		{
			directionOfMotion = Direction.BACKWARD;
		}
		else
		{
			directionOfMotion = lastPosChangeDirection.getEnumValue();
		}

		return directionOfMotion;
	}

	private boolean determineIfDirectionChanged(double currentPosition)
	{
		Direction currentDirection = computeDirectionOfMotion(currentPosition);

		boolean directionOfMotionChanged = lastPosChangeDirection.getEnumValue() != currentDirection;  lastPosChangeDirection.set(currentDirection);

		return directionOfMotionChanged;
	}


	private enum Direction
	{
		NONE, FORWARD, BACKWARD;
	}

}
