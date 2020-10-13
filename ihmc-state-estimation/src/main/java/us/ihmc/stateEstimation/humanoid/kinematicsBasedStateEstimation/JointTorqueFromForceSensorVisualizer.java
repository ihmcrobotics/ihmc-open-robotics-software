package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointTorqueFromForceSensorVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final List<RigidBodyBasics> allRigidBodies;
   private final Map<RigidBodyBasics, FootSwitchInterface> footSwitches;
   private final Map<RigidBodyBasics, GeometricJacobian> jacobians;
   private final Map<OneDoFJointBasics, YoDouble> jointTorques;

   public JointTorqueFromForceSensorVisualizer(Map<RigidBodyBasics, FootSwitchInterface> footSwitches, YoRegistry parentRegistry)
   {
      this.footSwitches = footSwitches;

      allRigidBodies = new ArrayList<>(footSwitches.keySet());
      jacobians = new HashMap<>();
      jointTorques = new HashMap<>();

      for (RigidBodyBasics rigidBody : allRigidBodies)
      {
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(rigidBody);
         OneDoFJointBasics[] oneDoFJoints = MultiBodySystemTools.createOneDoFJointPath(rootBody, rigidBody);

         GeometricJacobian jacobian = new GeometricJacobian(oneDoFJoints, rigidBody.getBodyFixedFrame());
         jacobians.put(rigidBody, jacobian);

         for (OneDoFJointBasics joint : oneDoFJoints)
         {
            if (!jointTorques.containsKey(joint))
            {
               String variableName = "tau_forceSensor_" + joint.getName();
               jointTorques.put(joint, new YoDouble(variableName, registry));
            }
         }
      }

      parentRegistry.addChild(registry);
   }

   private final Wrench wrench = new Wrench();
   private final DMatrixRMaj jointTorquesMatrix = new DMatrixRMaj(1, 1);

   public void update()
   {
      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = allRigidBodies.get(i);
         FootSwitchInterface footSwitch = footSwitches.get(rigidBody);
         GeometricJacobian jacobian = jacobians.get(rigidBody);

         footSwitch.computeAndPackFootWrench(wrench);
         wrench.changeFrame(rigidBody.getBodyFixedFrame());
         wrench.negate();

         jacobian.compute();
         jacobian.computeJointTorques(wrench, jointTorquesMatrix);

         JointBasics[] joints = jacobian.getJointsInOrder();

         for (int j = 0; j < joints.length; j++)
         {
            OneDoFJointBasics joint = (OneDoFJointBasics) joints[j];
            jointTorques.get(joint).set(jointTorquesMatrix.get(j, 0));
         }
      }
   }
}
