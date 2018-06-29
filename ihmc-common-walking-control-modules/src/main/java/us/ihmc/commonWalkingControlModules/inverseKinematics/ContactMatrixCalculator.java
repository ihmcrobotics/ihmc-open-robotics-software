package us.ihmc.commonWalkingControlModules.inverseKinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.robotics.screwTheory.*;

import java.util.List;

public class ContactMatrixCalculator
{
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F fullTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final FloatingInverseDynamicsJoint rootJoint;

   private final JointIndexHandler jointIndexHandler;

   public ContactMatrixCalculator(WholeBodyControlCoreToolbox toolbox)
   {
      jointIndexHandler = toolbox.getJointIndexHandler();
      rootJoint = toolbox.getRootJoint();
   }

   // FIXME this could be completely incorrect
   public boolean convertPlaneContactStateCommand(PlaneContactStateCommand command, MotionQPInput motionQPInputToPack)
   {
      if (rootJoint == null || command.isEmpty())
         return false;

      ConstraintType constraintType = ConstraintType.OBJECTIVE;
      motionQPInputToPack.reshape(6);
      motionQPInputToPack.setConstraintType(constraintType);
      fullTaskJacobian.reshape(motionQPInputToPack.taskJacobian.getNumRows(), motionQPInputToPack.taskJacobian.getNumCols());

      // If the task is setup as a hard constraint, there is no need for a weight matrix.
      if (constraintType == ConstraintType.OBJECTIVE)
      {
         // Compute the M-by-M weight matrix W computed as follows: W = S * W * S^T
         double weight = 150.0;
         motionQPInputToPack.setUseWeightScalar(true);
         motionQPInputToPack.setWeight(weight);
      }

      // get the Jacobian from the end effector to the base
      RigidBody base = rootJoint.getSuccessor();
      RigidBody endEffector = command.getContactingRigidBody();

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(base, endEffector);
      jacobianCalculator.setJacobianFrame(rootJoint.getPredecessor().getBodyFixedFrame());
      jacobianCalculator.computeJacobianMatrix();

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      jacobianCalculator.getJacobianMatrix(tempTaskJacobian);

      // Dealing with the primary base:
      List<InverseDynamicsJoint> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, motionQPInputToPack.taskJacobian);


      // get the Jacobian from the base to the world
      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(rootJoint.getPredecessor(), rootJoint.getSuccessor());
      jacobianCalculator.setJacobianFrame(rootJoint.getPredecessor().getBodyFixedFrame());
      jacobianCalculator.computeJacobianMatrix();

      jacobianCalculator.getJacobianMatrix(tempTaskJacobian);

      // Dealing with the primary base:
      jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, fullTaskJacobian);

      CommonOps.addEquals(motionQPInputToPack.taskJacobian, fullTaskJacobian);
      // We're done!

      return true;
   }
}
