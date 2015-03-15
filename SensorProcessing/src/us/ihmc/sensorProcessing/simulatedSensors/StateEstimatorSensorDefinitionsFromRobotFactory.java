package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;

public class StateEstimatorSensorDefinitionsFromRobotFactory
{
   private final SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap;
   private final LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions;
   private final Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions;

   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;

   public StateEstimatorSensorDefinitionsFromRobotFactory(SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap, ArrayList<IMUMount> imuMounts,
         ArrayList<WrenchCalculatorInterface> forceSensors)
   {
      this.scsToInverseDynamicsJointMap = scsToInverseDynamicsJointMap;
      this.imuDefinitions = generateIMUDefinitions(imuMounts);
      this.forceSensorDefinitions = generateForceSensorDefinitions(forceSensors);

      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      createAndAddForceSensorDefinitions(forceSensorDefinitions);
      createAndAddOneDoFSensors();
      createAndAddIMUSensors(imuDefinitions);
   }

   private void createAndAddForceSensorDefinitions(Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions.values())
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }
   }

   private LinkedHashMap<WrenchCalculatorInterface, ForceSensorDefinition> generateForceSensorDefinitions(
         ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCalculators)
   {
      LinkedHashMap<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions = new LinkedHashMap<WrenchCalculatorInterface, ForceSensorDefinition>();
      for (WrenchCalculatorInterface groundContactPointBasedWrenchCalculator : groundContactPointBasedWrenchCalculators)
      {
         OneDegreeOfFreedomJoint forceTorqueSensorJoint = groundContactPointBasedWrenchCalculator.getJoint();
         OneDoFJoint sensorParentJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(forceTorqueSensorJoint);
         ForceSensorDefinition sensorDefinition = new ForceSensorDefinition(groundContactPointBasedWrenchCalculator.getName(),
               sensorParentJoint.getSuccessor(), new RigidBodyTransform());
         forceSensorDefinitions.put(groundContactPointBasedWrenchCalculator, sensorDefinition);

      }
      return forceSensorDefinitions;
   }

   public Map<IMUMount, IMUDefinition> getIMUDefinitions()
   {
      return imuDefinitions;
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   private LinkedHashMap<IMUMount, IMUDefinition> generateIMUDefinitions(ArrayList<IMUMount> imuMounts)
   {
      LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions = new LinkedHashMap<IMUMount, IMUDefinition>();

      for (IMUMount imuMount : imuMounts)
      {
         RigidBody rigidBody = scsToInverseDynamicsJointMap.getRigidBody(imuMount.getParentJoint());
         RigidBodyTransform transformFromMountToJoint = new RigidBodyTransform();
         imuMount.getTransformFromMountToJoint(transformFromMountToJoint);
         IMUDefinition imuDefinition = new IMUDefinition(imuMount.getName(), rigidBody, transformFromMountToJoint);
         imuDefinitions.put(imuMount, imuDefinition);
      }

      return imuDefinitions;
   }

   public void createAndAddOneDoFSensors()
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>(
            scsToInverseDynamicsJointMap.getSCSOneDegreeOfFreedomJoints());

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(oneDegreeOfFreedomJoint);

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
