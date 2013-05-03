package us.ihmc.sensorProcessing.simulatedSensors;

public interface ControllerDispatcher
{
   public void startEstimator(long timestamp);
   public void waitUntilComputationIsDone();
}
