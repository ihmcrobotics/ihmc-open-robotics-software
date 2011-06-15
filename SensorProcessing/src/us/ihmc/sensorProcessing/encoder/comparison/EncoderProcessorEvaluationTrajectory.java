package us.ihmc.sensorProcessing.encoder.comparison;

public interface EncoderProcessorEvaluationTrajectory
{
   public abstract void update(double time);
   public abstract double getPosition();
   public abstract double getVelocity();
}
