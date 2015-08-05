package us.ihmc.atlas.drcsimGazebo;

import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.humanoidRobotics.model.DesiredJointDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class DRCSimGazeboSensorReaderFactory implements SensorReaderFactory
{
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private DRCRobotSensorInformation sensorInformation;
   private DRCSimGazeboSensorReader atlasSensorReader;
   private final StateEstimatorParameters stateEstimatorParameters;

   public DRCSimGazeboSensorReaderFactory(DRCRobotSensorInformation sensorInformation,
         StateEstimatorParameters stateEstimatorParameters)
   {
      this.sensorInformation = sensorInformation;     
      this.stateEstimatorParameters = stateEstimatorParameters;
   }

   public void build(SixDoFJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
         ContactSensorHolder contactSensorDataHolder, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, DesiredJointDataHolder estimatorDesiredJointDataHolder, YoVariableRegistry registry)
   {
      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
         }
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }

      atlasSensorReader = new DRCSimGazeboSensorReader(stateEstimatorSensorDefinitions, sensorInformation, stateEstimatorParameters,
            rawJointSensorDataHolderMap, registry);
   }

   public DRCSimGazeboSensorReader getSensorReader()
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
