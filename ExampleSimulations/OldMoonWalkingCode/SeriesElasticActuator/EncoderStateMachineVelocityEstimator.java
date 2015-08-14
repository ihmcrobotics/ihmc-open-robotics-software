package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.YoVariable;

public interface EncoderStateMachineVelocityEstimator
{
    abstract void update();
    abstract YoVariable getProcessedPosition();
    abstract YoVariable getProcessedRate();
}
