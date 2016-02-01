package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.YoVariableType;

public class EncoderStateMachineVelocityEstimatorSimple implements EncoderStateMachineVelocityEstimator
{
    private final YoVariable rawPosition, changeInTime;
    private final YoVariable processedPosition, processedRate, changeInPosition;

    private final YoVariable previousPosition, previousTime;

    private final YoVariable time;

    private boolean hasBeenCalled;

    public EncoderStateMachineVelocityEstimatorSimple(String name, YoVariable rawPosition, YoVariable time, Robot robot)
    {
	this.rawPosition = rawPosition;
	this.time = time;

	name = name + "_";
	
	YoVariableRegistry registry = new YoVariableRegistry(name);

	this.processedPosition = new YoVariable(name + "procPos", registry);
	this.processedRate = new YoVariable(name + "procRate", registry);

	this.previousPosition = new YoVariable(name + "prevPos", YoVariableType.INT, registry);
	this.previousTime = new YoVariable(name + "prevTime", registry);
	this.changeInPosition = new YoVariable(name + "changeInPos", YoVariableType.INT, registry);
	this.changeInTime = new YoVariable(name + "changeInTime", registry);
	
	
	
	if (robot != null)
	    robot.addYoVariableRegistry(registry);

	hasBeenCalled = false;
    }

    public YoVariable getProcessedPosition()
    {
	return processedPosition;
    }

    public YoVariable getProcessedRate()
    {
	return processedRate;
    }

    public void update()
    {
	if (!hasBeenCalled)
	{
	    setPreviousValues();
	    hasBeenCalled = true;
	}
	
	changeInPosition.val = rawPosition.getIntegerValue() - previousPosition.getIntegerValue();
	changeInTime.val = time.getDoubleValue() - previousTime.getDoubleValue();
	
	
	processedPosition.val = rawPosition.getIntegerValue();
	
	if (changeInTime.val != 0.0)
	    processedRate.val = ((double)changeInPosition.getIntegerValue())/changeInTime.getDoubleValue();
	else
	    processedRate.val = 0.0;
	
	setPreviousValues();
    }
    
    private void setPreviousValues()
    {
	previousPosition.val = rawPosition.getIntegerValue();
	previousTime.val = time.getDoubleValue();
    }

}
