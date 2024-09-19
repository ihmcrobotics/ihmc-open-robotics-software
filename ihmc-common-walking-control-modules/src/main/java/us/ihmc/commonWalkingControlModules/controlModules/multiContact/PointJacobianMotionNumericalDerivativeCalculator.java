package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

import java.util.ArrayList;
import java.util.List;

public class PointJacobianMotionNumericalDerivativeCalculator
{
   private static final boolean DEBUG = false;
   private static final int LINEAR_MOTION_DOFS = 3;
   private static final double DT_INTEGRATION = 1e-6;

   private final List<OneDoFJointBasics> oneDoFJoints = new ArrayList<>();
   private final TDoubleArrayList nominalJointAngles = new TDoubleArrayList();

   private int jointIndex;
   private boolean containsFloatingRootJoint;

   /* 3 x n_j point-jacobian motion derivative */
   private final DMatrixRMaj jacobianMotionDerivative = new DMatrixRMaj(0);
   /* 6 x n_j geometric jacobian of the nominal configuration */
   private final DMatrixRMaj nominalJacobian = new DMatrixRMaj(0);
   /* 6 x n_j geometric jacobian-rate matrix, computed through finite difference */
   private final DMatrixRMaj jacobianRateMatrix = new DMatrixRMaj(0);

   /**
    * Computes the 3 x n point-jacobian motion derivative of column j of the given jacobian, where j corresponds to the given joint and n is the number of 1dof joints.
    * Floating root joint is ignored if in the chain.
    * @return jacobian motion (derivative partial J / partial q).
    */
   public DMatrixRMaj compute(GeometricJacobian jacobian, OneDoFJointBasics joint, Runnable jacobianFrameUpdateCallback)
   {
      JointBasics[] joints = jacobian.getJointsInOrder();
      if (!initialize(joints, joint))
         return null;

      jacobianMotionDerivative.reshape(LINEAR_MOTION_DOFS, oneDoFJoints.size());
      jacobianRateMatrix.reshape(SpatialVectorReadOnly.SIZE, oneDoFJoints.size());

      if (DEBUG)
      {
         LogTools.info("Computing rates wrt joint " + joint.getName());
      }

      jacobian.compute();
      nominalJacobian.set(jacobian.getJacobianMatrix());

      for (int joint_idx = 0; joint_idx < oneDoFJoints.size(); joint_idx++)
      {
         setJointState(joint_idx);
         joints[0].getPredecessor().updateFramesRecursively();
         jacobianFrameUpdateCallback.run();

         jacobian.compute();
         jacobianRateMatrix.set(jacobian.getJacobianMatrix());
         CommonOps_DDRM.subtractEquals(jacobianRateMatrix, nominalJacobian);
         CommonOps_DDRM.scale(1.0 / DT_INTEGRATION, jacobianRateMatrix);

         if (DEBUG)
         {
            System.out.println("Jacobian rate matrix, joint " + oneDoFJoints.get(joint_idx).getName());
            System.out.println(jacobianRateMatrix);
            System.out.println("------------------------------------------------------------------------------------");
         }

         for (int linearMotionIdx = 0; linearMotionIdx < LINEAR_MOTION_DOFS; linearMotionIdx++)
         {
            int columnOffset = containsFloatingRootJoint ? SpatialVectorReadOnly.SIZE : 0;
            double jacobianMotionDerivativeEntry = jacobianRateMatrix.get(LINEAR_MOTION_DOFS + linearMotionIdx, columnOffset + jointIndex);
            jacobianMotionDerivative.set(linearMotionIdx, joint_idx, jacobianMotionDerivativeEntry);
         }
      }

      // reset nominal joint state
      setJointState(-1);

      return jacobianMotionDerivative;
   }

   private boolean initialize(JointBasics[] joints, OneDoFJointBasics joint)
   {
      oneDoFJoints.clear();
      jointIndex = -1;
      containsFloatingRootJoint = false;
      nominalJointAngles.reset();
      int first1DoFIndex = 0;

      if (joints[0] instanceof FloatingJointBasics floatingRootJoint)
      {
         containsFloatingRootJoint = true;
         first1DoFIndex = 1;
      }

      for (int jointIdx = first1DoFIndex; jointIdx < joints.length; jointIdx++)
      {
         if (joints[jointIdx] == joint)
            jointIndex = jointIdx - first1DoFIndex;

         OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joints[jointIdx];
         oneDoFJoints.add(oneDoFJoint);
         nominalJointAngles.add(oneDoFJoint.getQ());
      }

      return jointIndex >= 0;
   }

   private void setJointState(int jointIndexToIntegratePosition)
   {
      for (int jointIndex = 0; jointIndex < oneDoFJoints.size(); jointIndex++)
      {
         double q = nominalJointAngles.get(jointIndex);
         if (jointIndex == jointIndexToIntegratePosition)
            q += DT_INTEGRATION;
         oneDoFJoints.get(jointIndex).setQ(q);
      }
   }
}
