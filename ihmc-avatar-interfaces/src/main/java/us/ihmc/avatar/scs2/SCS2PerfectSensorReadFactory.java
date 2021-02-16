package us.ihmc.avatar.scs2;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2PerfectSensorReadFactory implements SensorReaderFactory
{
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private ForceSensorDataHolder forceSensorDataHolderToUpdate;

   private final ControllerInput controllerInput;
   private final DRCPerfectSensorReader perfectSensorReader;

   public SCS2PerfectSensorReadFactory(ControllerInput controllerInput)
   {
      this.controllerInput = controllerInput;
      perfectSensorReader = new DRCPerfectSensorReader(0);
   }

   @Override
   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolderToUpdate)
   {
      this.forceSensorDataHolderToUpdate = forceSensorDataHolderToUpdate;
   }

   @Override
   public void build(FloatingJointBasics rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
                     JointDesiredOutputListBasics estimatorDesiredJointDataHolder, YoRegistry parentRegistry)
   {
      
   }

   @Override
   public SensorReader getSensorReader()
   {
      return null;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public boolean useStateEstimator()
   {
      return false;
   }
}
