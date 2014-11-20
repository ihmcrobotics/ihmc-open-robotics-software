package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.LinkedHashMap;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


/**
 * JointStateUpdater simply reads the joint position/velocity sensors and updates the FullInverseDynamicsStructure.
 * (Based on {@link us.ihmc.sensorProcessing.stateEstimation.JointStateFullRobotModelUpdater}.)
 * @author Sylvain
 *
 */
public class JointStateUpdater
{
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBody rootBody;

   private final OneDoFJoint[] oneDoFJoints;
   private final SensorOutputMapReadOnly sensorMap;
   private IMUBasedPelvisToTorsoEncodersVelocityFilter iMUBasedPelvisToTorsoEncodersVelocityFilter;

   private boolean USE_SPINE_JOINT_SMOOTHENER = true;

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly, StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      rootBody = twistCalculator.getRootBody();
      
      this.sensorMap = sensorOutputMapReadOnly;
  
      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);

      if (USE_SPINE_JOINT_SMOOTHENER ){
         setupSpineJointVelocitiesSmooothener(sensorOutputMapReadOnly, stateEstimatorParameters, parentRegistry);
      }
   }

   public void setupSpineJointVelocitiesSmooothener(SensorOutputMapReadOnly sensorOutputMapReadOnly, StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      if (stateEstimatorParameters == null || stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation() == null)
         return;

      IMUSensorReadOnly pelvisIMU = null;
      IMUSensorReadOnly chestIMU = null;
      
      for (int i = 0; i < sensorOutputMapReadOnly.getIMUProcessedOutputs().size(); i++)
      {
         IMUSensorReadOnly sensorReadOnly = sensorOutputMapReadOnly.getIMUProcessedOutputs().get(i);
         if (sensorReadOnly.getSensorName().equals(stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation().first()))
            pelvisIMU = sensorReadOnly;
         
         if (sensorReadOnly.getSensorName().equals(stateEstimatorParameters.getIMUsForSpineJointVelocityEstimation().second()))
            chestIMU = sensorReadOnly;
      }
      
      // TODO create the module with the two IMUs to compute and smoothen the spine joint velocities here.
      iMUBasedPelvisToTorsoEncodersVelocityFilter = new IMUBasedPelvisToTorsoEncodersVelocityFilter(parentRegistry, pelvisIMU, chestIMU, sensorOutputMapReadOnly);
      iMUBasedPelvisToTorsoEncodersVelocityFilter.compute();
   }

   public void initialize()
   {
      updateJointState();
   }

   public void updateJointState()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         
         double positionSensorData = sensorMap.getJointPositionProcessedOutput(oneDoFJoint);
         double velocitySensorData = sensorMap.getJointVelocityProcessedOutput(oneDoFJoint);

         oneDoFJoint.setQ(positionSensorData);
         oneDoFJoint.setQd(velocitySensorData);
      }

      rootBody.updateFramesRecursively();
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();

      if (USE_SPINE_JOINT_SMOOTHENER)
      {
         if (iMUBasedPelvisToTorsoEncodersVelocityFilter != null)
         iMUBasedPelvisToTorsoEncodersVelocityFilter.compute();
      }

   }
}
