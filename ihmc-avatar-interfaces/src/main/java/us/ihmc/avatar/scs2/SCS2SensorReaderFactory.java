package us.ihmc.avatar.scs2;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2SensorReaderFactory implements SensorReaderFactory
{
   private ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private SCS2SensorReader sensorReader;

   private final SimControllerInput controllerInput;
   private final SensorProcessingConfiguration sensorProcessingConfiguration;
   private final boolean usePerfectSensors;

   public static SCS2SensorReaderFactory newSensorReaderFactory(SimControllerInput controllerInput, SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      return new SCS2SensorReaderFactory(controllerInput, sensorProcessingConfiguration, false);
   }

   public static SCS2SensorReaderFactory newPerfectSensorReaderFactory(SimControllerInput controllerInput)
   {
      return new SCS2SensorReaderFactory(controllerInput, null, true);
   }

   private SCS2SensorReaderFactory(SimControllerInput controllerInput, SensorProcessingConfiguration sensorProcessingConfiguration, boolean usePerfectSensors)
   {
      this.controllerInput = controllerInput;
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;
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
      if (usePerfectSensors)
         sensorReader = SCS2SensorReader.newPerfectSensorReader(controllerInput, rootJoint);
      else
         sensorReader = SCS2SensorReader.newSensorReader(controllerInput,
                                                         rootJoint,
                                                         imuDefinitions,
                                                         forceSensorDataHolderToUpdate,
                                                         sensorProcessingConfiguration);
      parentRegistry.addChild(sensorReader.getRegistry());
   }

   @Override
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return sensorReader.getStateEstimatorSensorDefinitions();
   }

   @Override
   public boolean useStateEstimator()
   {
      return !usePerfectSensors;
   }
}
