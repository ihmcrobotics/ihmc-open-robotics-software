package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface SensorReaderFactory
{
   public abstract void build(FloatingJointBasics rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
                              JointDesiredOutputList estimatorDesiredJointDataHolder, YoVariableRegistry parentRegistry);

   public abstract SensorReader getSensorReader();

   public abstract StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions();

   public abstract boolean useStateEstimator();

}