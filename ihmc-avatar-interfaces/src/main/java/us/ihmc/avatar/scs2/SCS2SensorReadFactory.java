package us.ihmc.avatar.scs2;

import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2SensorReadFactory implements SensorReaderFactory
{
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private SCS2SensorReader perfectSensorReader;

   private final SimControllerInput controllerInput;
   private final boolean usePerfectSensors;

   public SCS2SensorReadFactory(SimControllerInput controllerInput, boolean usePerfectSensors)
   {
      this.controllerInput = controllerInput;
      this.usePerfectSensors = usePerfectSensors;
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
      perfectSensorReader = new SCS2SensorReader(controllerInput, rootJoint, usePerfectSensors);
      Stream.of(imuDefinitions).forEach(perfectSensorReader::addIMUSensor);
      perfectSensorReader.addWrenchSensors(forceSensorDataHolderToUpdate);
   }

   @Override
   public SensorReader getSensorReader()
   {
      return perfectSensorReader;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public boolean useStateEstimator()
   {
      return !usePerfectSensors;
   }
}
