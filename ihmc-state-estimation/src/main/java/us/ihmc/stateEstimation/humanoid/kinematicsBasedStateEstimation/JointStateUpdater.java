package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * JointStateUpdater simply reads the joint position/velocity sensors and updates the
 * FullInverseDynamicsStructure.
 * 
 * @author Sylvain
 */
public class JointStateUpdater
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private OneDoFJointBasics[] oneDoFJoints;
   private final SensorOutputMapReadOnly sensorMap;
   private final List<IMUBasedJointStateEstimator> imuBasedJointStateEstimators;

   private RigidBodyBasics rootBody;

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure,
                            SensorOutputMapReadOnly sensorOutputMapReadOnly,
                            StateEstimatorParameters stateEstimatorParameters,
                            YoRegistry parentRegistry)
   {
      rootBody = inverseDynamicsStructure.getElevator();
      this.sensorMap = sensorOutputMapReadOnly;

      JointBasics[] joints = MultiBodySystemTools.collectSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = MultiBodySystemTools.filterJoints(joints, OneDoFJointBasics.class);

      imuBasedJointStateEstimators = createIMUBasedJointVelocityEstimators(sensorOutputMapReadOnly, stateEstimatorParameters, registry);

      parentRegistry.addChild(registry);
   }

   public void setJointsToUpdate(OneDoFJointBasics[] oneDoFJoints)
   {
      this.oneDoFJoints = oneDoFJoints;
   }

   public List<IMUBasedJointStateEstimator> createIMUBasedJointVelocityEstimators(SensorOutputMapReadOnly sensorOutputMapReadOnly,
                                                                                  StateEstimatorParameters stateEstimatorParameters,
                                                                                  YoRegistry parentRegistry)
   {
      if (stateEstimatorParameters == null)
         return Collections.emptyList();

      List<IMUBasedJointStateEstimator> estimators = new ArrayList<>();

      for (IMUBasedJointStateEstimatorParameters parameters : stateEstimatorParameters.getIMUBasedJointStateEstimatorParameters())
      {
         IMUSensorReadOnly parentIMU = null;
         IMUSensorReadOnly childIMU = null;

         String parentIMUName = parameters.getParentIMUName();
         String childIMUName = parameters.getChildIMUName();

         for (int i = 0; i < sensorOutputMapReadOnly.getIMUOutputs().size(); i++)
         {
            IMUSensorReadOnly sensorReadOnly = sensorOutputMapReadOnly.getIMUOutputs().get(i);
            if (sensorReadOnly.getSensorName().equals(parentIMUName))
               parentIMU = sensorReadOnly;

            if (sensorReadOnly.getSensorName().equals(childIMUName))
               childIMU = sensorReadOnly;
         }

         // TODO create the module with the two IMUs to compute and smoothen the spine joint velocities here.
         if (parentIMU != null && childIMU != null)
         {
            estimators.add(new IMUBasedJointStateEstimator(stateEstimatorParameters.getEstimatorDT(),
                                                           parentIMU,
                                                           childIMU,
                                                           sensorOutputMapReadOnly,
                                                           parameters,
                                                           parentRegistry));
         }
         else
         {
            LogTools.warn("Could not find the given parent and/or child IMUs: parentIMU = " + parentIMUName + ", childIMU = " + childIMUName);
            if (parentIMU == null)
               LogTools.warn("Parent IMU is null.");

            if (childIMU == null)
               LogTools.warn("Child IMU is null.");
         }
      }

      return estimators;
   }

   public void initialize()
   {
      updateJointState();
   }

   public void updateJointState()
   {
      if (imuBasedJointStateEstimators != null)
      {
         for (int i = 0; i < imuBasedJointStateEstimators.size(); i++)
         {
            imuBasedJointStateEstimators.get(i).compute();
         }
      }

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics oneDoFJoint = oneDoFJoints[i];
         OneDoFJointStateReadOnly jointSensorOutput = sensorMap.getOneDoFJointOutput(oneDoFJoint);

         double positionSensorData = jointSensorOutput.getPosition();
         double velocitySensorData = jointSensorOutput.getVelocity();
         double torqueSensorData = jointSensorOutput.getEffort();

         for (int j = 0; j < imuBasedJointStateEstimators.size(); j++)
         {
            IMUBasedJointStateEstimator estimator = imuBasedJointStateEstimators.get(j);

            if (!estimator.containsJoint(oneDoFJoint))
               continue;

            double estimatedJointPosition = estimator.getEstimatedJointPosition(oneDoFJoint);
            if (!Double.isNaN(estimatedJointPosition))
               positionSensorData = estimatedJointPosition;

            double estimatedJointVelocity = estimator.getEstimatedJointVelocity(oneDoFJoint);
            if (!Double.isNaN(estimatedJointVelocity))
               velocitySensorData = estimatedJointVelocity;

            break; // Stop at the first estimator that has a value for this joint.
         }

         oneDoFJoint.setQ(positionSensorData);
         oneDoFJoint.setQd(velocitySensorData);
         oneDoFJoint.setTau(torqueSensorData);
      }

      rootBody.updateFramesRecursively();
   }
}
