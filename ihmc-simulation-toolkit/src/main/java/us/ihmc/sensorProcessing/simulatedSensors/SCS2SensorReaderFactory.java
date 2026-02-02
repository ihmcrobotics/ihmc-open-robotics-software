package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2SensorReaderFactory implements SensorReaderFactory
{
   private ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private SCS2SensorReader sensorReader;

   private final SimControllerInput controllerInput;
   private final StateEstimatorParameters stateEstimatorParameters;
   private final boolean usePerfectSensors;

   public static SCS2SensorReaderFactory newSensorReaderFactory(SimControllerInput controllerInput, StateEstimatorParameters stateEstimatorParameters)
   {
      return new SCS2SensorReaderFactory(controllerInput, stateEstimatorParameters, false);
   }

   public static SCS2SensorReaderFactory newPerfectSensorReaderFactory(SimControllerInput controllerInput)
   {
      return new SCS2SensorReaderFactory(controllerInput, new StateEstimatorParameters(), true);
   }

   private SCS2SensorReaderFactory(SimControllerInput controllerInput, StateEstimatorParameters stateEstimatorParameters, boolean usePerfectSensors)
   {
      this.controllerInput = controllerInput;
      this.stateEstimatorParameters = stateEstimatorParameters;
      this.usePerfectSensors = usePerfectSensors;
   }

   @Override
   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolderToUpdate)
   {
      this.forceSensorDataHolderToUpdate = forceSensorDataHolderToUpdate;
   }

   @Override
   public void build(FloatingJointBasics rootJoint,
                     IMUDefinition[] imuDefinitions,
                     ForceSensorDefinition[] forceSensorDefinitions,
                     JointDesiredOutputListBasics estimatorDesiredJointDataHolder,
                     YoRegistry parentRegistry)
   {
      if (usePerfectSensors)
         sensorReader = SCS2SensorReader.newPerfectSensorReader(controllerInput, rootJoint, imuDefinitions, forceSensorDataHolderToUpdate);
      else
         sensorReader = SCS2SensorReader.newSensorReader(controllerInput,
                                                         rootJoint,
                                                         imuDefinitions,
                                                         forceSensorDataHolderToUpdate,
                                                         stateEstimatorParameters);
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
