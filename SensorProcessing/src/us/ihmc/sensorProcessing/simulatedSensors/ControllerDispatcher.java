package us.ihmc.sensorProcessing.simulatedSensors;

public interface ControllerDispatcher
{
   public void startEstimator(long timestamp, long estimatorClockStartTime);
   public void waitUntilComputationIsDone();
}
