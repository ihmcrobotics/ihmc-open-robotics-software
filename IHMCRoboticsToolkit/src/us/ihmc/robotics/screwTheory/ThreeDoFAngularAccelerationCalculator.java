package us.ihmc.robotics.screwTheory;

import org.ejml.alg.dense.mult.SubmatrixOps;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FrameVector;

public class ThreeDoFAngularAccelerationCalculator
{
   private final GeometricJacobian jacobian;
   private final DesiredJointAccelerationCalculator jointAccelerationCalculator;
   private final int sign;

   public ThreeDoFAngularAccelerationCalculator(RigidBody base, RigidBody endEffector)
   {
      RigidBody ancestor;
      RigidBody descendant;
      
      if (ScrewTools.isAncestor(endEffector, base))
      {
         ancestor = base;
         descendant = endEffector;
         sign = +1;
      }
      else
      {
         ancestor = endEffector;
         descendant = base;
         sign = -1;
      }
      
      this.jacobian = new GeometricJacobian(ancestor, descendant, descendant.getBodyFixedFrame());
      this.jointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, null);
   }

   public void compute(FrameVector desiredAngularAcceleration)
   {
      int vectorSize = SpatialMotionVector.SIZE / 2;
      DenseMatrix64F biasedAccelerationMatrix = new DenseMatrix64F(vectorSize, 1);
      DenseMatrix64F jacobianDerivativeTermMatrix = new DenseMatrix64F(vectorSize, 1);
      DenseMatrix64F angularJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE / 2, jacobian.getNumberOfColumns());
      DenseMatrix64F jointAccelerations = new DenseMatrix64F(angularJacobian.getNumCols(), 1);

      desiredAngularAcceleration.changeFrame(jacobian.getJacobianFrame());
      desiredAngularAcceleration.getInMatrixColumn(biasedAccelerationMatrix, 0);
      CommonOps.scale(sign, biasedAccelerationMatrix);

      jacobian.compute();
      SpatialAccelerationVector jacobianDerivativeTerm = new SpatialAccelerationVector();
      jointAccelerationCalculator.computeJacobianDerivativeTerm(jacobianDerivativeTerm);
      jacobianDerivativeTerm.getAngularPart().get(jacobianDerivativeTermMatrix);
      CommonOps.subtractEquals(biasedAccelerationMatrix, jacobianDerivativeTermMatrix);

      SubmatrixOps.setSubMatrix(jacobian.getJacobianMatrix(), angularJacobian, 0, 0, 0, 0, angularJacobian.getNumRows(), angularJacobian.getNumCols());

      CommonOps.solve(angularJacobian, biasedAccelerationMatrix, jointAccelerations);

      ScrewTools.setDesiredAccelerations(jacobian.getJointsInOrder(), jointAccelerations);
   }
}
