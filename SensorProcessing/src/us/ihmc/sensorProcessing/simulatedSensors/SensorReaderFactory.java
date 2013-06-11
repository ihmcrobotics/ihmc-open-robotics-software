package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public interface SensorReaderFactory
{

   public abstract void build(SixDoFJoint sixDoFJoint, IMUDefinition[] imuDefinitions, boolean addLinearAccelerationSensors);

   public abstract SensorReader getSensorReader();

   public abstract StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions();
   
   public abstract boolean useStateEstimator();

}