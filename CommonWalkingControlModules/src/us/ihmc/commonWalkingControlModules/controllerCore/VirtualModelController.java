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
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final RigidBody defaultRootBody;

   private Map<InverseDynamicsJoint, Double> jointTorques = new HashMap<>();

   private final ArrayList<RigidBody> endEffectors = new ArrayList<>();
   private final HashMap<RigidBody, RigidBody> baseOfEndEffector = new HashMap<>();
   private final HashMap<RigidBody, Long> endEffectorJacobians = new HashMap<>();
   private final HashMap<RigidBody, Wrench> endEffectorWrenches = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> endEffectorSelectionMatrices = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> jointEffortMatrices = new HashMap<>();

   private final DenseMatrix64F tmpWrenchMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tmpEffortMatrix = new DenseMatrix64F(1, 1);


   public VirtualModelController(GeometricJacobianHolder geometricJacobianHolder, RigidBody defaultRootBody)
   {
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.defaultRootBody = defaultRootBody;
   }

   public void registerEndEffector(RigidBody endEffector)
   {
      registerEndEffector(defaultRootBody, endEffector);
   }

   public void registerEndEffector(RigidBody base, RigidBody endEffector)
   {
      if (!endEffectors.contains(endEffector) && endEffector != null)
      {
         endEffectors.add(endEffector);
         baseOfEndEffector.put(endEffector, base);

         OneDoFJoint[] joints = ScrewTools.createOneDoFJointPath(base, endEffector);

         long jacobianID = geometricJacobianHolder.getOrCreateGeometricJacobian(joints, base.getBodyFixedFrame());
         endEffectorJacobians.put(endEffector, jacobianID);

         tmpEffortMatrix.reshape(joints.length, 1);
         tmpEffortMatrix.zero();
         jointEffortMatrices.put(endEffector, tmpEffortMatrix);
      }
   }

   private final Wrench dummyWrench = new Wrench();
   private final DenseMatrix64F forceSelectionMatrix = new DenseMatrix64F(3, Wrench.SIZE);
   public void submitEndEffectorVirtualForce(RigidBody endEffector, FrameVector force)
   {
      dummyWrench.setToZero();
      dummyWrench.setToZero(baseOfEndEffector.get(endEffector).getBodyFixedFrame(), force.getReferenceFrame());
      dummyWrench.setLinearPart(force);

      forceSelectionMatrix.set(0, 3, 1);
      forceSelectionMatrix.set(1, 4, 1);
      forceSelectionMatrix.set(2, 5, 1);

      endEffectorWrenches.put(endEffector, dummyWrench);
      endEffectorSelectionMatrices.put(endEffector, forceSelectionMatrix);
   }

   public void submitEndEffectorVirtualForce(RigidBody endEffector, FrameVector force, DenseMatrix64F selectionMatrix)
   {
      dummyWrench.setToZero();
      dummyWrench.setToZero(baseOfEndEffector.get(endEffector).getBodyFixedFrame(), force.getReferenceFrame());
      dummyWrench.setLinearPart(force);

      forceSelectionMatrix.set(0, 3, 1);
      forceSelectionMatrix.set(1, 4, 1);
      forceSelectionMatrix.set(2, 5, 1);

      endEffectorWrenches.put(endEffector, dummyWrench);
      endEffectorSelectionMatrices.put(endEffector, forceSelectionMatrix);
   }

   public void submitEndEffectorVirtualWrench(RigidBody endEffector, Wrench wrench)
   {
      wrench.changeFrame(baseOfEndEffector.get(endEffector).getBodyFixedFrame());
      wrench.changeBodyFrameAttachedToSameBody(endEffector.getBodyFixedFrame());

      endEffectorWrenches.put(endEffector, wrench);
      endEffectorSelectionMatrices.put(endEffector, CommonOps.identity(Wrench.SIZE));
   }

   public void submitEndEffectorVirtualWrench(RigidBody endEffector, Wrench wrench, DenseMatrix64F selectionMatrix)
   {
      //wrench.changeFrame(baseOfEndEffector.get(endEffector).getBodyFixedFrame());
      //wrench.changeBodyFrameAttachedToSameBody(endEffector.getBodyFixedFrame());

      endEffectorWrenches.put(endEffector, wrench);
      endEffectorSelectionMatrices.put(endEffector, selectionMatrix);
   }

   public void submitEndEffectorVirtualWrench(VirtualWrenchCommand virtualWrenchCommand)
   {
      submitEndEffectorVirtualWrench(virtualWrenchCommand.getRigidBody(), virtualWrenchCommand.getVirtualWrench(), virtualWrenchCommand.getSelectionMatrix());
   }

   public void reset()
   {
      endEffectorWrenches.clear();
      endEffectorSelectionMatrices.clear();
      jointTorques.clear();
   }


   public final DenseMatrix64F tmpEndEffectorWrench = new DenseMatrix64F(1, 1);
   public final DenseMatrix64F tmpJacobianMatrix = new DenseMatrix64F(1, 1);
   public final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);

   public void compute(VirtualModelControlSolution virtualModelControlSolutionToPack)
   {
      for (RigidBody endEffector : endEffectors)
      {
         if (!endEffectorWrenches.containsKey(endEffector) || !endEffectorSelectionMatrices.containsKey(endEffector))
            throw new RuntimeException("Not all registered end effectors have required forces to compute desired joint torques.");
      }

      for (RigidBody endEffector : endEffectors)
      {
         DenseMatrix64F endEffectorSelectionMatrix = endEffectorSelectionMatrices.get(endEffector);
         int taskSize = endEffectorSelectionMatrix.numRows;

         // check and set frames
         GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(endEffectorJacobians.get(endEffector));
         Wrench endEffectorWrench = endEffectorWrenches.get(endEffector);
         endEffectorWrench.changeFrame(jacobian.getBaseFrame());
         endEffectorWrench.changeBodyFrameAttachedToSameBody(jacobian.getJacobianFrame());

         DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
         endEffectorWrench.getMatrix(wrenchMatrix);
         DenseMatrix64F jointEffortMatrix = jointEffortMatrices.get(endEffector);
         jointEffortMatrix.zero();

         // Apply selection matrix
         tmpEndEffectorWrench.reshape(taskSize, 1);
         tmpJacobianMatrix.reshape(taskSize, jacobianMatrix.numCols);
         CommonOps.mult(endEffectorSelectionMatrix, wrenchMatrix, tmpEndEffectorWrench);
         CommonOps.mult(endEffectorSelectionMatrix, jacobianMatrix, tmpJacobianMatrix);

         // Compute desired joint torques
         DenseMatrix64F altJointEffortMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
         altJointEffortMatrix = jacobian.computeJointTorques(endEffectorWrench);
         CommonOps.multTransA(tmpJacobianMatrix, tmpEndEffectorWrench, jointEffortMatrix);

         // Write torques to map
         int index = 0;
         for (InverseDynamicsJoint joint : jacobian.getJointsInOrder())
         {
            jointTorques.put(joint, jointEffortMatrix.get(index));
            index++;
         }
      }

      virtualModelControlSolutionToPack.setJointTorques(jointTorques);
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
