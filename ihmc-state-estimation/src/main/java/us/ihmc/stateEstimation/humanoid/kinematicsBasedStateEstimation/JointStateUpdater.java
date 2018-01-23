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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

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
   private final IMUBasedJointVelocityEstimator iMUBasedJointVelocityEstimator;

   private final YoBoolean enableIMUBasedJointVelocityEstimator = new YoBoolean("enableIMUBasedJointVelocityEstimator", registry);

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      rootBody = inverseDynamicsStructure.getElevator();

      this.sensorMap = sensorOutputMapReadOnly;

      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);

      iMUBasedJointVelocityEstimator = createIMUBasedJointVelocityEstimator(sensorOutputMapReadOnly, stateEstimatorParameters, registry);

      parentRegistry.addChild(registry);
   }

   public void setJointsToUpdate(OneDoFJoint[] oneDoFJoints)
   {
      this.oneDoFJoints = oneDoFJoints;
   }

   public IMUBasedJointVelocityEstimator createIMUBasedJointVelocityEstimator(SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      if (stateEstimatorParameters == null || stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation() == null)
         return null;

      enableIMUBasedJointVelocityEstimator.set(stateEstimatorParameters.useIMUsForSpineJointVelocityEstimation());

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
         double estimatorDT = stateEstimatorParameters.getEstimatorDT();
         double slopTime = stateEstimatorParameters.getIMUJointVelocityEstimationBacklashSlopTime();
         IMUBasedJointVelocityEstimator iMUBasedJointVelocityEstimator = new IMUBasedJointVelocityEstimator(pelvisIMU, chestIMU, sensorOutputMapReadOnly, estimatorDT, slopTime, parentRegistry);
         iMUBasedJointVelocityEstimator.compute();
         double alphaIMUsForSpineJointVelocityEstimation = stateEstimatorParameters.getAlphaIMUsForSpineJointVelocityEstimation();
         double alphaIMUsForSpineJointPositionEstimation = stateEstimatorParameters.getAlphaIMUsForSpineJointPositionEstimation();
         iMUBasedJointVelocityEstimator.setAlphaFuse(alphaIMUsForSpineJointVelocityEstimation, alphaIMUsForSpineJointPositionEstimation);
         return iMUBasedJointVelocityEstimator;
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
      if (iMUBasedJointVelocityEstimator != null)
      {
         iMUBasedJointVelocityEstimator.compute();
      }

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];

         double positionSensorData = sensorMap.getJointPositionProcessedOutput(oneDoFJoint);
         double velocitySensorData = sensorMap.getJointVelocityProcessedOutput(oneDoFJoint);
         double torqueSensorData = sensorMap.getJointTauProcessedOutput(oneDoFJoint);
         boolean jointEnabledIndicator = sensorMap.isJointEnabled(oneDoFJoint);

         if (enableIMUBasedJointVelocityEstimator.getBooleanValue() && iMUBasedJointVelocityEstimator != null)
         {
            double estimatedJointVelocity = iMUBasedJointVelocityEstimator.getEstimatedJointVelocitiy(oneDoFJoint);
            if (!Double.isNaN(estimatedJointVelocity))
               velocitySensorData = estimatedJointVelocity;

            double estimatedJointPosition = iMUBasedJointVelocityEstimator.getEstimatedJointPosition(oneDoFJoint);
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
