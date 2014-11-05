package us.ihmc.steppr.hardware.sensorReader;

import java.io.IOException;
import java.util.HashMap;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprSensorReaderFactory implements SensorReaderFactory
{
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private StepprSensorReader sensorReader;

   private final DRCRobotSensorInformation sensorInformation;
   private final SensorFilterParameters sensorFilterParameters;
   private final SensorNoiseParameters sensorNoiseParameters;

   public StepprSensorReaderFactory(DRCRobotModel robotModel)
   {
      sensorInformation = robotModel.getSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();
      sensorNoiseParameters = stateEstimatorParameters.getSensorNoiseParameters();
      sensorFilterParameters = stateEstimatorParameters.getSensorFilterParameters();
   }

   @Override
   public void build(SixDoFJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
         ForceSensorDataHolder forceSensorDataHolderForEstimator, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, YoVariableRegistry parentRegistry)
   {
      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      HashMap<String, OneDoFJoint> allJoints = new HashMap<String, OneDoFJoint>();
      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            stateEstimatorSensorDefinitions.addJointPositionSensorDefinition(oneDoFJoint);
            stateEstimatorSensorDefinitions.addJointVelocitySensorDefinition(oneDoFJoint);
            allJoints.put(oneDoFJoint.getName(), oneDoFJoint);
         }
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         stateEstimatorSensorDefinitions.addOrientationSensorDefinition(imuDefinition);
         stateEstimatorSensorDefinitions.addAngularVelocitySensorDefinition(imuDefinition);
         stateEstimatorSensorDefinitions.addLinearAccelerationSensorDefinition(imuDefinition);
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }

      sensorReader = new StepprSensorReader(stateEstimatorSensorDefinitions, forceSensorDataHolderForEstimator, rawJointSensorDataHolderMap, sensorInformation,
            sensorFilterParameters, sensorNoiseParameters, parentRegistry);

   }

   @Override
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   public void connect() throws IOException
   {
      sensorReader.connect();
   }

   public void disconnect()
   {
      sensorReader.disconnect();
   }

   @Override
   public boolean useStateEstimator()
   {
      return true;
   }

}
