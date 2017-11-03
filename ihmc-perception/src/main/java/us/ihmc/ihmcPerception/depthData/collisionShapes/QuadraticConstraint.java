package us.ihmc.ihmcPerception.depthData.collisionShapes;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class QuadraticConstraint extends AbstractConstraint
{
   private DenseMatrix64F P;
   private DenseMatrix64F Q;
   private DenseMatrix64F R;
   
   private DenseMatrix64F tempVectorForCalc;
   
   public QuadraticConstraint(int n, DenseMatrix64F P, DenseMatrix64F Q, DenseMatrix64F R, ConstraintType type)
   {
      super(n, type);
      this.P = P;
      this.Q = Q;
      this.R = R;
      this.tempVectorForCalc = new DenseMatrix64F(n, 1);
   }

   @Override
   public double getConstraintViolation(DenseMatrix64F xVector)
   {
      CommonOps.mult(P, xVector, tempVectorForCalc); // P * x
      CommonOps.multTransA(xVector, tempVectorForCalc, constraintViolation); // x^T * P * x
      CommonOps.mult(Q, xVector, tempVectorForCalc); // Q * x
      CommonOps.addEquals(constraintViolation, tempVectorForCalc); // x^T * P * x + Q * x
      CommonOps.addEquals(constraintViolation, R); // x^T * P * x + Q * x + R
      
      return this.constraintViolation.get(0, 0);
   }

   @Override
   public DenseMatrix64F getConstraintJacobian(DenseMatrix64F xVector)
   {
      CommonOps.multTransA(xVector, P, constraintJacobian);
      CommonOps.addEquals(constraintJacobian, Q);
      return constraintJacobian;
   }
}
