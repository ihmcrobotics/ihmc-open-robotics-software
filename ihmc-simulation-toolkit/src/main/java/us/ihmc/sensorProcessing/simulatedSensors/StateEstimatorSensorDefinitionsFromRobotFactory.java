package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class StateEstimatorSensorDefinitionsFromRobotFactory
{
   private final SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap;
   private final LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions;
   private final Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions;

   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;

   public StateEstimatorSensorDefinitionsFromRobotFactory(SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap, ArrayList<IMUMount> imuMounts,
         ArrayList<WrenchCalculatorInterface> forceSensors, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions)
   {
      this.scsToInverseDynamicsJointMap = scsToInverseDynamicsJointMap;
      this.imuDefinitions = generateIMUDefinitions(imuMounts, imuDefinitions);
      this.forceSensorDefinitions = generateForceSensorDefinitions(forceSensors, forceSensorDefinitions);

      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      createAndAddForceSensorDefinitions(this.forceSensorDefinitions);
      createAndAddOneDoFSensors();
      createAndAddIMUSensors(this.imuDefinitions);
   }

   private void createAndAddForceSensorDefinitions(Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions.values())
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }
   }

   private LinkedHashMap<WrenchCalculatorInterface, ForceSensorDefinition> generateForceSensorDefinitions(ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCalculators,
                                                                                                          ForceSensorDefinition[] forceSensorDefinitions)
   {
      LinkedHashMap<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitionMap = new LinkedHashMap<WrenchCalculatorInterface, ForceSensorDefinition>();
      for (WrenchCalculatorInterface groundContactPointBasedWrenchCalculator : groundContactPointBasedWrenchCalculators)
      {
         ForceSensorDefinition forceSensorDefinition = null;
         for (int i = 0; i < forceSensorDefinitions.length; i++)
         {
            if (forceSensorDefinitions[i].getSensorName().equals(groundContactPointBasedWrenchCalculator.getName()))
            {
               forceSensorDefinition = forceSensorDefinitions[i];
               break;
            }
         }
         if (forceSensorDefinition == null)
         {
            throw new RuntimeException("Could not find force sensor definition for " + groundContactPointBasedWrenchCalculator.getName());
         }

         forceSensorDefinitionMap.put(groundContactPointBasedWrenchCalculator, forceSensorDefinition);
      }
      return forceSensorDefinitionMap;
   }

   public Map<IMUMount, IMUDefinition> getIMUDefinitions()
   {
      return imuDefinitions;
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   private LinkedHashMap<IMUMount, IMUDefinition> generateIMUDefinitions(ArrayList<IMUMount> imuMounts, IMUDefinition[] imuDefinitions)
   {
      LinkedHashMap<IMUMount, IMUDefinition> imuDefinitionMap = new LinkedHashMap<IMUMount, IMUDefinition>();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = null;
         for (int i = 0; i < imuDefinitions.length; i++)
         {
            if (imuDefinitions[i].getName().equals(imuMount.getName()))
            {
               imuDefinition = imuDefinitions[i];
               break;
            }
         }
         if (imuDefinition == null)
         {
            throw new RuntimeException("Could not find imu definition for " + imuMount.getName());
         }

         imuDefinitionMap.put(imuMount, imuDefinition);
      }

      return imuDefinitionMap;
   }

   public void createAndAddOneDoFSensors()
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>(
            scsToInverseDynamicsJointMap.getSCSOneDegreeOfFreedomJoints());

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJointBasics oneDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(oneDegreeOfFreedomJoint);

         stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
      }
   }

   public void createAndAddIMUSensors(LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }
   }

   public Map<WrenchCalculatorInterface, ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }
}
