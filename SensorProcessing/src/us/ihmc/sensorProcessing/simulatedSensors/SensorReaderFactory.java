package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public interface SensorReaderFactory
{
   public abstract void build(SixDoFJoint rootJoint, IMUDefinition[] imuDefinitions, boolean addLinearAccelerationSensors, YoVariableRegistry parentRegistry);

   public abstract SensorReader getSensorReader();

   public abstract StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions();
   
   public abstract boolean useStateEstimator();

}