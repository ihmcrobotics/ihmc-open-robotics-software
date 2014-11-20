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
   private final static boolean DEBUG = false;
   
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBody rootBody;

   private final OneDoFJoint[] oneDoFJoints;
   private final SensorOutputMapReadOnly sensorMap;
   private final YoVariableRegistry registry;
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> estimatedJointPositions;
   private IMUBasedPelvisToTorsoEncodersVelocityFilter iMUBasedPelvisToTorsoEncodersVelocityFilter;

   private boolean USE_SPINE_JOINT_SMOOTHENER = true;

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly, StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      rootBody = twistCalculator.getRootBody();
      
      this.sensorMap = sensorOutputMapReadOnly;
  
      // TODO Enable that guy when ready
//      setupSpineJointVelocitiesSmooothener(sensorOutputMapReadOnly, stateEstimatorParameters);
      
      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);

      if (DEBUG)
      {
         estimatedJointPositions = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
         registry = new YoVariableRegistry(getClass().getSimpleName());
         
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint oneDoFJoint = oneDoFJoints[i];
            DoubleYoVariable jointPosition = new DoubleYoVariable("estimated_q_" + oneDoFJoint.getName(), registry);
            estimatedJointPositions.put(oneDoFJoint, jointPosition);
         }
         
         parentRegistry.addChild(registry);
      }
      else
      {
         estimatedJointPositions = null;
         registry = null;
      }
      
      if (USE_SPINE_JOINT_SMOOTHENER ){
         setupSpineJointVelocitiesSmooothener(sensorOutputMapReadOnly, stateEstimatorParameters);
      }
   }

   public void setupSpineJointVelocitiesSmooothener(SensorOutputMapReadOnly sensorOutputMapReadOnly, StateEstimatorParameters stateEstimatorParameters)
   {
      if (stateEstimatorParameters == null)
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
      iMUBasedPelvisToTorsoEncodersVelocityFilter = new IMUBasedPelvisToTorsoEncodersVelocityFilter(registry, pelvisIMU, chestIMU, sensorOutputMapReadOnly);
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

         if (DEBUG)
            estimatedJointPositions.get(oneDoFJoint).set(positionSensorData);
         
         oneDoFJoint.setQ(positionSensorData);
         oneDoFJoint.setQd(velocitySensorData);
      }

      rootBody.updateFramesRecursively();
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      
      if (USE_SPINE_JOINT_SMOOTHENER){
         iMUBasedPelvisToTorsoEncodersVelocityFilter.compute();
      }
      
   }
}
