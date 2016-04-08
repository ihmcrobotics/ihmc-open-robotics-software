package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.*;

import java.util.ArrayList;
import java.util.HashMap;

public class VirtualModelController
{
   private final WholeBodyControlCoreToolbox toolbox;
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final RigidBody rootBody;

   private final ArrayList<RigidBody> endEffectors = new ArrayList<>();
   private final HashMap<RigidBody, Long> endEffectorJacobians = new HashMap<>();
   private final HashMap<RigidBody, OneDoFJoint[]> limbJoints = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> endEffectorWrenchMatrices = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> endEffectorSelectionMatrices = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> jointEffortMatrices = new HashMap<>();

   private final DenseMatrix64F tmpWrenchMatrix = new DenseMatrix64F();
   private final DenseMatrix64F tmpEffortMatrix = new DenseMatrix64F();


   public VirtualModelController(WholeBodyControlCoreToolbox toolbox)
   {
      this.toolbox = toolbox;
      rootBody = toolbox.getRobotRootJoint().getSuccessor();
      geometricJacobianHolder = toolbox.getGeometricJacobianHolder();
   }

   public void registerEndEffector(RigidBody endEffector)
   {
      if (!endEffectors.contains(endEffector))
      {
         endEffectors.add(endEffector);

         OneDoFJoint[] joints = ScrewTools.filterJoints(ScrewTools.createJointPath(rootBody, endEffector), OneDoFJoint.class);
         limbJoints.put(endEffector, joints);

         long jacobianID = geometricJacobianHolder.getOrCreateGeometricJacobian(joints, rootBody.getBodyFixedFrame());
         endEffectorJacobians.put(endEffector, jacobianID);

         tmpEffortMatrix.reshape(limbJoints.size(), 1);
         tmpEffortMatrix.zero();
         jointEffortMatrices.put(endEffector, tmpEffortMatrix);
      }
   }

   public void submitEndEffectorVirtualForce(RigidBody endEffector, FrameVector force)
   {
      if (limbJoints.get(endEffector).length != 3)
         throw new RuntimeException("This limb requires a wrench to be full rank.");

      tmpWrenchMatrix.reshape(3, 1);
      tmpWrenchMatrix.set(0, 0, force.getX());
      tmpWrenchMatrix.set(1, 0, force.getY());
      tmpWrenchMatrix.set(2, 0, force.getZ());

      endEffectorWrenchMatrices.put(endEffector, tmpWrenchMatrix);
      endEffectorSelectionMatrices.put(endEffector, CommonOps.identity(3));
   }

   public void submitEndEffectorVirtualForce(RigidBody endEffector, FrameVector force, DenseMatrix64F selectionMatrix)
   {
      if (limbJoints.get(endEffector).length != selectionMatrix.numRows)
         throw new RuntimeException("This limb does not have the number of joints to achieve this end effector force.");

      tmpWrenchMatrix.reshape(3, 1);
      tmpWrenchMatrix.set(0, 0, force.getX());
      tmpWrenchMatrix.set(1, 0, force.getY());
      tmpWrenchMatrix.set(2, 0, force.getZ());

      endEffectorWrenchMatrices.put(endEffector, tmpWrenchMatrix);
      endEffectorSelectionMatrices.put(endEffector, selectionMatrix);
   }

   public void submitEndEffectorVirtualWrench(RigidBody endEffector, Wrench wrench)
   {
      if (limbJoints.get(endEffector).length != SpatialAccelerationVector.SIZE)
         throw new RuntimeException("This limb does not have the number of joints to achieve this end effector force.");

      tmpWrenchMatrix.reshape(SpatialAccelerationVector.SIZE, 1);
      tmpWrenchMatrix.set(0, 0, wrench.getAngularPartX());
      tmpWrenchMatrix.set(1, 0, wrench.getAngularPartY());
      tmpWrenchMatrix.set(2, 0, wrench.getAngularPartZ());
      tmpWrenchMatrix.set(3, 0, wrench.getLinearPartX());
      tmpWrenchMatrix.set(4, 0, wrench.getLinearPartY());
      tmpWrenchMatrix.set(5, 0, wrench.getLinearPartZ());

      endEffectorWrenchMatrices.put(endEffector, tmpWrenchMatrix);
      endEffectorSelectionMatrices.put(endEffector, CommonOps.identity(SpatialAccelerationVector.SIZE));
   }

   public void submitEndEffectorVirtualWrench(RigidBody endEffector, Wrench wrench, DenseMatrix64F selectionMatrix)
   {
      if (limbJoints.get(endEffector).length != selectionMatrix.numRows)
         throw new RuntimeException("This limb does not have the number of joints to achieve this end effector force.");

      tmpWrenchMatrix.reshape(SpatialAccelerationVector.SIZE, 1);
      tmpWrenchMatrix.set(0, 0, wrench.getAngularPartX());
      tmpWrenchMatrix.set(1, 0, wrench.getAngularPartY());
      tmpWrenchMatrix.set(2, 0, wrench.getAngularPartZ());
      tmpWrenchMatrix.set(3, 0, wrench.getLinearPartX());
      tmpWrenchMatrix.set(4, 0, wrench.getLinearPartY());
      tmpWrenchMatrix.set(5, 0, wrench.getLinearPartZ());

      endEffectorWrenchMatrices.put(endEffector, tmpWrenchMatrix);
      endEffectorSelectionMatrices.put(endEffector, selectionMatrix);
   }


   public void reset()
   {
      endEffectorWrenchMatrices.clear();
      endEffectorSelectionMatrices.clear();
   }

   public void compute()
   {
      for (RigidBody endEffector : endEffectors)
      {
         if (!endEffectorWrenchMatrices.containsKey(endEffector) || !endEffectorSelectionMatrices.containsKey(endEffector))
            throw new RuntimeException("Not all registered end effectors have required forces to compute desired torques.");
      }

      for (RigidBody endEffector : endEffectors)
      {
         DenseMatrix64F jacobianMatrix = geometricJacobianHolder.getJacobian(endEffectorJacobians.get(endEffector)).getJacobianMatrix();
         DenseMatrix64F endEffectorWrenchMatrix = endEffectorWrenchMatrices.get(endEffector);
         DenseMatrix64F endEffectorSelectionMatrix = endEffectorSelectionMatrices.get(endEffector);
         DenseMatrix64F jointEffortMatrix = jointEffortMatrices.get(endEffector);
         jointEffortMatrix.zero();

         // Fix dimensionality of desired force to be achievable with the limb joints.
         tmpWrenchMatrix.reshape(endEffectorSelectionMatrix.numRows, endEffectorWrenchMatrix.numRows);
         CommonOps.mult(endEffectorSelectionMatrix, endEffectorWrenchMatrix, tmpWrenchMatrix);

         // Compute desired joint torques
         CommonOps.multTransA(jacobianMatrix, tmpWrenchMatrix, jointEffortMatrix);

         OneDoFJoint[] joints = limbJoints.get(endEffector);
      }
   }
}
