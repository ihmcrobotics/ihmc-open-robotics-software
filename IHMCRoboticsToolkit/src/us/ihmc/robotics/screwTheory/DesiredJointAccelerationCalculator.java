package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

public class DesiredJointAccelerationCalculator
{
   private final DenseMatrix64F jacobianDerivativeTermMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F jointAccelerations;
   private final DenseMatrix64F biasedSpatialAcceleration = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final SpatialAccelerationVector jacobianDerivativeTerm = new SpatialAccelerationVector();

   private final LinearSolver<DenseMatrix64F> jacobianSolver;
   private final ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();
   private final GeometricJacobian jacobian;

   public DesiredJointAccelerationCalculator(GeometricJacobian jacobian, LinearSolver<DenseMatrix64F> jacobianSolver)
   {
      this.jointAccelerations = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jacobian.getJointsInOrder()), 1);
      this.jacobian = jacobian;
      this.jacobianSolver = jacobianSolver;
   }

   /**
    * Sets the accelerations for the RevoluteJoints in legJoints
    * Assumes that the swingLegJacobian is already updated
    * Assumes that the rootJoint's acceleration has already been set
    */
   public void compute(SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBase)
   {
      convectiveTermCalculator.computeJacobianDerivativeTerm(jacobian, jacobianDerivativeTerm);
      computeJointAccelerations(accelerationOfEndEffectorWithRespectToBase, jacobianDerivativeTerm);
   }

   public void computeJacobianDerivativeTerm(SpatialAccelerationVector accelerationToPack)
   {
      convectiveTermCalculator.computeJacobianDerivativeTerm(jacobian, accelerationToPack);
   }

   private void computeJointAccelerations(SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBase, SpatialAccelerationVector jacobianDerivativeTerm)
   {
      accelerationOfEndEffectorWithRespectToBase.getBodyFrame().checkReferenceFrameMatch(jacobianDerivativeTerm.getBodyFrame());
      accelerationOfEndEffectorWithRespectToBase.getBaseFrame().checkReferenceFrameMatch(jacobianDerivativeTerm.getBaseFrame());
      accelerationOfEndEffectorWithRespectToBase.getExpressedInFrame().checkReferenceFrameMatch(jacobianDerivativeTerm.getExpressedInFrame());
      jacobian.getJacobianFrame().checkReferenceFrameMatch(accelerationOfEndEffectorWithRespectToBase.getExpressedInFrame());

      accelerationOfEndEffectorWithRespectToBase.packMatrix(biasedSpatialAcceleration, 0);    // unbiased at this point
      jacobianDerivativeTerm.packMatrix(jacobianDerivativeTermMatrix, 0);
      CommonOps.subtractEquals(biasedSpatialAcceleration, jacobianDerivativeTermMatrix);
      if (!jacobianSolver.setA(jacobian.getJacobianMatrix()))
         throw new RuntimeException("jacobian cannot be solved");
      jacobianSolver.solve(biasedSpatialAcceleration, jointAccelerations);

      ScrewTools.setDesiredAccelerations(jacobian.getJointsInOrder(), jointAccelerations);
   }
}
