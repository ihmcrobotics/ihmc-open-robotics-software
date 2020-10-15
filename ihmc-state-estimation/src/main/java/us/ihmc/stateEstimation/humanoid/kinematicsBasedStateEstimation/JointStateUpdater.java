package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * JointStateUpdater simply reads the joint position/velocity sensors and updates the FullInverseDynamicsStructure.
 * @author Sylvain
 *
 */
public class JointStateUpdater
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final RigidBodyBasics rootBody;

   private OneDoFJointBasics[] oneDoFJoints;
   private final SensorOutputMapReadOnly sensorMap;
   private final IMUBasedJointStateEstimator iMUBasedJointStateEstimator;

   private final BooleanProvider enableIMUBasedJointVelocityEstimator;

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters, YoRegistry parentRegistry)
   {
      rootBody = inverseDynamicsStructure.getElevator();

      this.sensorMap = sensorOutputMapReadOnly;

      JointBasics[] joints = MultiBodySystemTools.collectSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = MultiBodySystemTools.filterJoints(joints, OneDoFJointBasics.class);

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

   public void setJointsToUpdate(OneDoFJointBasics[] oneDoFJoints)
   {
      this.oneDoFJoints = oneDoFJoints;
   }

   public IMUBasedJointStateEstimator createIMUBasedJointVelocityEstimator(SensorOutputMapReadOnly sensorOutputMapReadOnly,
                                                                           StateEstimatorParameters stateEstimatorParameters, YoRegistry parentRegistry)
   {
      if (stateEstimatorParameters == null || stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation() == null)
         return null;

      IMUSensorReadOnly pelvisIMU = null;
      IMUSensorReadOnly chestIMU = null;

      String pelvisIMUName = stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation().getLeft();
      String chestIMUName = stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation().getRight();

      for (int i = 0; i < sensorOutputMapReadOnly.getIMUOutputs().size(); i++)
      {
         IMUSensorReadOnly sensorReadOnly = sensorOutputMapReadOnly.getIMUOutputs().get(i);
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
         LogTools.warn("Could not find the given pelvis and/or chest IMUs: pelvisIMU = " + pelvisIMUName + ", chestIMU = " + chestIMUName);
         if(pelvisIMU == null)
         {
            LogTools.warn("Pelvis IMU is null.");
         }

         if(chestIMU == null)
         {
            LogTools.warn("Chest IMU is null.");
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
         OneDoFJointBasics oneDoFJoint = oneDoFJoints[i];
         OneDoFJointStateReadOnly jointSensorOutput = sensorMap.getOneDoFJointOutput(oneDoFJoint);

         double positionSensorData = jointSensorOutput.getPosition();
         double velocitySensorData = jointSensorOutput.getVelocity();
         double torqueSensorData = jointSensorOutput.getEffort();

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
         oneDoFJoint.setTau(torqueSensorData);
      }

      rootBody.updateFramesRecursively();
   }
}
