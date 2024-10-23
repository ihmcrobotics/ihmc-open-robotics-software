package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface SensorReaderFactory
{
   public default void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder)
   {
   }

   public abstract void build(FloatingJointBasics rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
                              JointDesiredOutputListBasics estimatorDesiredJointDataHolder, YoRegistry parentRegistry);

   public abstract SensorReader getSensorReader();

   public abstract StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions();

   public abstract boolean useStateEstimator();

}