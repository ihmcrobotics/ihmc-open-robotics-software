package us.ihmc.atlas.drcsimGazebo;

import java.util.HashMap;

import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ContactSensorHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
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
         ForceSensorDataHolder forceSensorDataHolderForEstimator, ContactSensorHolder contactSensorDataHolder, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, YoVariableRegistry registry)
   {
      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      HashMap<String, OneDoFJoint> allJoints = new HashMap<String, OneDoFJoint>();
      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
            allJoints.put(oneDoFJoint.getName(), oneDoFJoint);
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
            forceSensorDataHolderForEstimator, rawJointSensorDataHolderMap, registry);
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
