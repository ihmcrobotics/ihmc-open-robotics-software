package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * JointStateUpdater simply reads the joint position/velocity sensors and updates the FullInverseDynamicsStructure.
 * @author Sylvain
 *
 */
public class JointStateUpdater
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBody rootBody;

   private OneDoFJoint[] oneDoFJoints;
   private final SensorOutputMapReadOnly sensorMap;
   private final IMUBasedJointStateEstimator iMUBasedJointStateEstimator;

   private final BooleanProvider enableIMUBasedJointVelocityEstimator;

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      rootBody = inverseDynamicsStructure.getElevator();

      this.sensorMap = sensorOutputMapReadOnly;

      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);

      if (stateEstimatorParameters == null)
      {
         enableIMUBasedJointVelocityEstimator = new BooleanParameter("enableIMUBasedJointVelocityEstimator", registry);
      }
      else
      {
         boolean initialValue = stateEstimatorParameters.useIMUsForSpineJointVelocityEstimation();
         enableIMUBasedJointVelocityEstimator = new BooleanParameter("enableIMUBasedJointVelocityEstimator", registry, initialValue);
      }
      iMUBasedJointStateEstimator = createIMUBasedJointVelocityEstimator(sensorOutputMapReadOnly, stateEstimatorParameters, registry);

      parentRegistry.addChild(registry);
   }

   public void setJointsToUpdate(OneDoFJoint[] oneDoFJoints)
   {
      this.oneDoFJoints = oneDoFJoints;
   }

   public IMUBasedJointStateEstimator createIMUBasedJointVelocityEstimator(SensorOutputMapReadOnly sensorOutputMapReadOnly,
                                                                           StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      if (stateEstimatorParameters == null || stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation() == null)
         return null;

      IMUSensorReadOnly pelvisIMU = null;
      IMUSensorReadOnly chestIMU = null;

      String pelvisIMUName = stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation().getLeft();
      String chestIMUName = stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation().getRight();

      for (int i = 0; i < sensorOutputMapReadOnly.getIMUProcessedOutputs().size(); i++)
      {
         IMUSensorReadOnly sensorReadOnly = sensorOutputMapReadOnly.getIMUProcessedOutputs().get(i);
         if (sensorReadOnly.getSensorName().equals(pelvisIMUName))
            pelvisIMU = sensorReadOnly;

         if (sensorReadOnly.getSensorName().equals(chestIMUName))
            chestIMU = sensorReadOnly;
      }

      // TODO create the module with the two IMUs to compute and smoothen the spine joint velocities here.
      if (pelvisIMU != null && chestIMU != null)
      {
         return new IMUBasedJointStateEstimator(pelvisIMU, chestIMU, sensorOutputMapReadOnly, stateEstimatorParameters, parentRegistry);
      }
      else
      {
         PrintTools.warn("Could not find the given pelvis and/or chest IMUs: pelvisIMU = " + pelvisIMUName + ", chestIMU = " + chestIMUName);
         if(pelvisIMU == null)
         {
            PrintTools.warn("Pelvis IMU is null.");
         }

         if(chestIMU == null)
         {
            PrintTools.warn("Chest IMU is null.");
         }

         return null;
      }
   }

   public void initialize()
   {
      updateJointState();
   }

   public void updateJointState()
   {
      if (iMUBasedJointStateEstimator != null)
      {
         iMUBasedJointStateEstimator.compute();
      }

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];

         double positionSensorData = sensorMap.getJointPositionProcessedOutput(oneDoFJoint);
         double velocitySensorData = sensorMap.getJointVelocityProcessedOutput(oneDoFJoint);
         double torqueSensorData = sensorMap.getJointTauProcessedOutput(oneDoFJoint);
         boolean jointEnabledIndicator = sensorMap.isJointEnabled(oneDoFJoint);

         if (enableIMUBasedJointVelocityEstimator.getValue() && iMUBasedJointStateEstimator != null)
         {
            double estimatedJointVelocity = iMUBasedJointStateEstimator.getEstimatedJointVelocitiy(oneDoFJoint);
            if (!Double.isNaN(estimatedJointVelocity))
               velocitySensorData = estimatedJointVelocity;

            double estimatedJointPosition = iMUBasedJointStateEstimator.getEstimatedJointPosition(oneDoFJoint);
            if (!Double.isNaN(estimatedJointPosition))
               positionSensorData = estimatedJointPosition;
         }

         oneDoFJoint.setQ(positionSensorData);
         oneDoFJoint.setQd(velocitySensorData);
         oneDoFJoint.setTauMeasured(torqueSensorData);
         oneDoFJoint.setEnabled(jointEnabledIndicator);
      }

      rootBody.updateFramesRecursively();
      spatialAccelerationCalculator.compute();
   }
}
