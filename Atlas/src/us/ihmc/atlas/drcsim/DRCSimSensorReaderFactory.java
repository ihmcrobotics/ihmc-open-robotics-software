package us.ihmc.atlas.drcsim;

import java.util.HashMap;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class DRCSimSensorReaderFactory implements SensorReaderFactory
{
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private DRCRobotSensorInformation sensorInformation;
   private DRCSimSensorReader atlasSensorReader;
   private final SensorFilterParameters sensorFilterParameters;
   private final SensorNoiseParameters sensorNoiseParameters;

   public DRCSimSensorReaderFactory(DRCRobotSensorInformation sensorInformation,
         SensorFilterParameters sensorFilterParameters, SensorNoiseParameters sensorNoiseParameters)
   {
      this.sensorInformation = sensorInformation;     
      
      this.sensorFilterParameters = sensorFilterParameters;
      this.sensorNoiseParameters = sensorNoiseParameters;
      
   }

   public void build(SixDoFJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
         ForceSensorDataHolder forceSensorDataHolderForEstimator, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, YoVariableRegistry registry)
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

      atlasSensorReader = new DRCSimSensorReader(stateEstimatorSensorDefinitions, sensorInformation, sensorFilterParameters, sensorNoiseParameters,
            forceSensorDataHolderForEstimator, rawJointSensorDataHolderMap, registry);
   }

   public DRCSimSensorReader getSensorReader()
   {
      return atlasSensorReader;
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   public boolean useStateEstimator()
   {
      return true;
   }

}
