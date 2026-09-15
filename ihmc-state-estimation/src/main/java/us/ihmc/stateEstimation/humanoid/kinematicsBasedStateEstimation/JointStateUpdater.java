package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.jointLevel.OneDoFJointStateSource;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * JointStateUpdater reads the joint position/velocity sensors, overlays the estimates of an
 * optional {@link OneDoFJointStateSource} (NaN falls back to the raw sensor value), and updates the
 * FullInverseDynamicsStructure.
 *
 * <p>This class never computes its source: the owner must run the pre-filter's phase 1
 * ({@code computeJointState()}) before <em>every</em> call to {@link #updateJointState()} — note
 * that {@link #initialize()} calls it too.</p>
 *
 * @author Sylvain
 */
public class JointStateUpdater
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private OneDoFJointBasics[] oneDoFJoints;
   private final SensorOutputMapReadOnly sensorMap;
   private final OneDoFJointStateSource jointStateSource; // may be null: raw sensor pass-through

   private RigidBodyBasics rootBody;

   public JointStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure,
                            SensorOutputMapReadOnly sensorOutputMapReadOnly,
                            OneDoFJointStateSource jointStateSource,
                            YoRegistry parentRegistry)
   {
      rootBody = inverseDynamicsStructure.getElevator();
      this.sensorMap = sensorOutputMapReadOnly;
      this.jointStateSource = jointStateSource;

      JointBasics[] joints = MultiBodySystemTools.collectSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = MultiBodySystemTools.filterJoints(joints, OneDoFJointBasics.class);

      parentRegistry.addChild(registry);
   }

   public void setJointsToUpdate(OneDoFJointBasics[] oneDoFJoints)
   {
      this.oneDoFJoints = oneDoFJoints;
   }

   public void initialize()
   {
      updateJointState();
   }

   public void updateJointState()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics oneDoFJoint = oneDoFJoints[i];
         OneDoFJointStateReadOnly jointSensorOutput = sensorMap.getOneDoFJointOutput(oneDoFJoint);

         double positionSensorData = jointSensorOutput.getPosition();
         double velocitySensorData = jointSensorOutput.getVelocity();
         double torqueSensorData = jointSensorOutput.getEffort();

         if (jointStateSource != null && jointStateSource.containsJoint(oneDoFJoint))
         {
            double estimatedJointPosition = jointStateSource.getEstimatedJointPosition(oneDoFJoint);
            if (!Double.isNaN(estimatedJointPosition))
               positionSensorData = estimatedJointPosition;

            double estimatedJointVelocity = jointStateSource.getEstimatedJointVelocity(oneDoFJoint);
            if (!Double.isNaN(estimatedJointVelocity))
               velocitySensorData = estimatedJointVelocity;
         }

         oneDoFJoint.setQ(positionSensorData);
         oneDoFJoint.setQd(velocitySensorData);
         oneDoFJoint.setTau(torqueSensorData);
      }

      rootBody.updateFramesRecursively();
   }
}
