package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.YoVariableType;

public class SimulatedEncoder 
{
	private final double encoderTicksPerUnitOfPosition;
	private YoVariable positionFromEncoder;
	private YoVariable encoderTicks;
	
	
	public SimulatedEncoder(double encoderTicksPerUnitOfPosition)
	{
	    this(encoderTicksPerUnitOfPosition, null, "");
	}
	
	
	public SimulatedEncoder(double encoderTicksPerUnitOfPosition, Robot robot, String name)
	{
		if (encoderTicksPerUnitOfPosition <= 0.0)
			throw new RuntimeException("encoderTicksPerUnitOfPosition must be > 0.0");
		
		YoVariableRegistry registry = new YoVariableRegistry("simulatedEncoder_" + name);
		
		positionFromEncoder = new YoVariable("positionFromEncoder_" + name, registry);
		encoderTicks = new YoVariable("encoderTicks_" + name, YoVariableType.INT, registry);
		
		if (robot != null)
		    robot.addYoVariableRegistry(registry);
		
		this.encoderTicksPerUnitOfPosition = encoderTicksPerUnitOfPosition;
	}
	
	public void setActualPosition(double actualPosition)
	{
		encoderTicks.val = (int) Math.round(actualPosition * encoderTicksPerUnitOfPosition);
		positionFromEncoder.val = ((double) encoderTicks.getIntegerValue()) / encoderTicksPerUnitOfPosition;
	}
	
	public double getPositionFromEncoder()
	{
		return positionFromEncoder.getDoubleValue();
	}
	
	public int getEncoderTicks()
	{
		return encoderTicks.getIntegerValue();
	}
	
	public double converTicksToDistance(int ticks)
	{
	    return ((double) ticks)/encoderTicksPerUnitOfPosition;
	}
	
	public double converTicksToDistance(double ticks)
	{
	    return ticks/encoderTicksPerUnitOfPosition;
	}
}
