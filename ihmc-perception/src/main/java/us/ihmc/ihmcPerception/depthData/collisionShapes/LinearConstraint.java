package us.ihmc.ihmcPerception.depthData.collisionShapes;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class LinearConstraint extends AbstractConstraint
{
   private DenseMatrix64F A;
   private DenseMatrix64F b;
   
   public LinearConstraint(int numberOfVariables, ConstraintType type, DenseMatrix64F A, DenseMatrix64F b)
   {
      super(numberOfVariables, type);
      this.A = A;
      this.b = b;
      constraintJacobian.set(A);
   }
   
   public LinearConstraint(int numberOfVariables, DenseMatrix64F A, DenseMatrix64F b)
   {
      this(numberOfVariables, ConstraintType.INEQUALITY, A, b);
   }
   
   public LinearConstraint(int numberOfVariables, ConstraintType type, DenseMatrix64F A, double b)
   {
      this(numberOfVariables, type, A, new DenseMatrix64F(1, 1, true, b));
   }
   
   @Override
   public double getConstraintViolation(DenseMatrix64F xVector)
   {
      validateXVector(xVector);
      CommonOps.mult(A, xVector, constraintViolation);
      CommonOps.subtractEquals(constraintViolation, b);
      return this.constraintViolation.get(0, 0);
   }

   @Override
   public DenseMatrix64F getConstraintJacobian(DenseMatrix64F xVector)
   {
      return constraintJacobian;
   }
}
