package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class JointTorqueFromForceSensorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<RigidBody> allRigidBodies;
   private final Map<RigidBody, FootSwitchInterface> footSwitches;
   private final Map<RigidBody, GeometricJacobian> jacobians;
   private final Map<OneDoFJoint, DoubleYoVariable> jointTorques;

   public JointTorqueFromForceSensorVisualizer(Map<RigidBody, FootSwitchInterface> footSwitches, YoVariableRegistry parentRegistry)
   {
      this.footSwitches = footSwitches;

      allRigidBodies = new ArrayList<>(footSwitches.keySet());
      jacobians = new HashMap<>();
      jointTorques = new HashMap<>();

      for (RigidBody rigidBody : allRigidBodies)
      {
         RigidBody rootBody = ScrewTools.getRootBody(rigidBody);
         OneDoFJoint[] oneDoFJoints = ScrewTools.createOneDoFJointPath(rootBody, rigidBody);

         GeometricJacobian jacobian = new GeometricJacobian(oneDoFJoints, rigidBody.getBodyFixedFrame());
         jacobians.put(rigidBody, jacobian);

         for (OneDoFJoint joint : oneDoFJoints)
         {
            if (!jointTorques.containsKey(joint))
            {
               String variableName = "tau_forceSensor_" + joint.getName();
               jointTorques.put(joint, new DoubleYoVariable(variableName, registry));
            }
         }
      }

      parentRegistry.addChild(registry);
   }

   private final Wrench wrench = new Wrench();
   private final DenseMatrix64F jointTorquesMatrix = new DenseMatrix64F(1, 1);

   public void update()
   {
      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);
         FootSwitchInterface footSwitch = footSwitches.get(rigidBody);
         GeometricJacobian jacobian = jacobians.get(rigidBody);

         footSwitch.computeAndPackFootWrench(wrench);
         wrench.changeFrame(rigidBody.getBodyFixedFrame());
         wrench.negate();

         jacobian.compute();
         jacobian.computeJointTorques(wrench, jointTorquesMatrix);

         InverseDynamicsJoint[] joints = jacobian.getJointsInOrder();

         for (int j = 0; j < joints.length; j++)
         {
            OneDoFJoint joint = (OneDoFJoint) joints[j];
            jointTorques.get(joint).set(jointTorquesMatrix.get(j, 0));
         }
      }
   }
}
