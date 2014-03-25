package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;

public interface StateEstimatorParameters
{
   public abstract boolean isRunningOnRealRobot();
   
   public abstract SensorFilterParameters getSensorFilterParameters();

   public abstract boolean getAssumePerfectIMU();

   public abstract boolean useKinematicsBasedStateEstimator();

   public abstract PointMeasurementNoiseParameters getPointMeasurementNoiseParameters();
   
   public abstract SensorNoiseParameters getSensorNoiseParameters();

   public abstract double getEstimatorDT();
}
