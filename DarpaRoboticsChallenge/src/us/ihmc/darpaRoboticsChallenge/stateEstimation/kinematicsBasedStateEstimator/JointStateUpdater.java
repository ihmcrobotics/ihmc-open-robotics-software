package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.LinkedHashMap;

import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

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
   private final JointAndIMUSensorMap sensorMap;
   private final YoVariableRegistry registry;
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> estimatedJointPositions;

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, JointAndIMUSensorMap jointAndIMUSensorMap, YoVariableRegistry parentRegistry)
   {
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      rootBody = twistCalculator.getRootBody();
      
      this.sensorMap = jointAndIMUSensorMap;
      
      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         if (sensorMap.getJointPositionSensorPort(oneDoFJoint) == null)
            throw new RuntimeException("sensorMap.getJointPositionSensorPort(oneDoFJoint) == null. oneDoFJoint = " + oneDoFJoint);
         
         if (sensorMap.getJointVelocitySensorPort(oneDoFJoint) == null)
            throw new RuntimeException("sensorMap.getJointVelocitySensorPort(oneDoFJoint) == null. oneDoFJoint = " + oneDoFJoint);
      }
      
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
         
         double positionSensorData = sensorMap.getJointPositionSensorPort(oneDoFJoint).getData()[0];
         double velocitySensorData = sensorMap.getJointVelocitySensorPort(oneDoFJoint).getData()[0];

         if (DEBUG)
            estimatedJointPositions.get(oneDoFJoint).set(positionSensorData);
         
         oneDoFJoint.setQ(positionSensorData);
         oneDoFJoint.setQd(velocitySensorData);
      }

      rootBody.updateFramesRecursively();
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
   }
}
