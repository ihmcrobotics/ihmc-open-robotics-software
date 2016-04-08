package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class VirtualModelController
{
   private final WholeBodyControlCoreToolbox toolbox;
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final RigidBody rootBody;

   private OneDoFJoint[] jointsToCompute;
   private Map<OneDoFJoint, Double> jointTorques = new HashMap<>();

   private final ArrayList<RigidBody> endEffectors = new ArrayList<>();
   private final HashMap<RigidBody, Long> endEffectorJacobians = new HashMap<>();
   private final HashMap<RigidBody, OneDoFJoint[]> limbJoints = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> endEffectorWrenchMatrices = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> endEffectorSelectionMatrices = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> jointEffortMatrices = new HashMap<>();

   private final DenseMatrix64F tmpWrenchMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tmpEffortMatrix = new DenseMatrix64F(1, 1);


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

         OneDoFJoint[] joints = ScrewTools.createOneDoFJointPath(rootBody, endEffector);
         limbJoints.put(endEffector, joints);
         jointsToCompute = appendJoints(jointsToCompute, joints);

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

   public void submitEndEffectorVirtualWrench(VirtualWrenchCommand virtualWrenchCommand)
   {
      submitEndEffectorVirtualWrench(virtualWrenchCommand.getRigidBody(), virtualWrenchCommand.getVirtualWrench(), virtualWrenchCommand.getSelectionMatrix());
   }

   public void reset()
   {
      endEffectorWrenchMatrices.clear();
      endEffectorSelectionMatrices.clear();
      jointTorques.clear();
   }

   public VirtualModelControlSolution compute()
   {
      for (RigidBody endEffector : endEffectors)
      {
         if (!endEffectorWrenchMatrices.containsKey(endEffector) || !endEffectorSelectionMatrices.containsKey(endEffector))
            throw new RuntimeException("Not all registered end effectors have required forces to compute desired joint torques.");
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

         // Write torques to map
         int index = 0;
         for (OneDoFJoint joint : limbJoints.get(endEffector))
         {
            jointTorques.put(joint, jointEffortMatrix.get(index));
            index++;
         }
      }

      return new VirtualModelControlSolution(jointsToCompute, jointTorques);
   }

   private static OneDoFJoint[] appendJoints(OneDoFJoint[] currentJoints, OneDoFJoint[] newJoints)
   {
      int start;
      if (currentJoints != null)
         start = currentJoints.length;
      else
         start = 0;

      OneDoFJoint[] ret = new OneDoFJoint[start + newJoints.length];
      for (int i = 0; i < start; i++)
         ret[i] = currentJoints[i];
      for (int i = start; i < ret.length; i++)
         ret[i] = newJoints[i - start];

      return ret;
   }
}
