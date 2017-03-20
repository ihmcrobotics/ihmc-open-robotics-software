package us.ihmc.robotics.functionApproximation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SpecializedOps;

public class DampedLeastSquaresSolver implements LinearSolver<DenseMatrix64F>
{
   private final DenseMatrix64F A;
   private double alpha;
   private final DenseMatrix64F tempMatrix1;
   private final DenseMatrix64F tempMatrix2;
   private final LinearSolver<DenseMatrix64F> linearSolver;
   private final LinearSolver<DenseMatrix64F> linearSolverAlpha0;

   public DampedLeastSquaresSolver(int matrixSize)
   {
      this(matrixSize, 0);
   }
   public DampedLeastSquaresSolver(int matrixSize, double alpha)
   {
      this.A = new DenseMatrix64F(matrixSize, matrixSize);
      this.tempMatrix1 = new DenseMatrix64F(matrixSize, matrixSize);
      this.tempMatrix2 = new DenseMatrix64F(matrixSize, 1);
//      this.linearSolver = LinearSolverFactory.linear(matrixSize);
      this.linearSolver = LinearSolverFactory.symmPosDef(matrixSize);
      this.linearSolverAlpha0 = LinearSolverFactory.pseudoInverse(true);
      this.alpha = alpha;       
   }

   /**
    * Sets the damping factor to solve the damped least squares problem.
    * @param alpha
    */
   public void setAlpha(double alpha)
   {
      this.alpha = alpha;
   }

   /** {@inheritDoc} */
   @Override
   public boolean setA(DenseMatrix64F A)
   {
      this.A.set(A);

      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double quality()
   {
      return CommonOps.det(A);
   }

   /** {@inheritDoc}
    * <p>
    *    In this instance, the problem is formulated as a damped least squares problem, with the damping factor set by {@link #setAlpha(double)}
    * </p>
    * <p>
    *    min x^T alpha^2 x + (Ax - B)^T (Ax - B)
    * </p>
    */
   @Override
   public void solve(DenseMatrix64F b, DenseMatrix64F x)
   {
      if (alpha == 0)
      {
         linearSolverAlpha0.setA(this.A);
         linearSolverAlpha0.solve(b, x);
      }
      else
      {
         tempMatrix1.reshape(A.getNumRows(), A.getNumRows());
         CommonOps.multTransB(A, A, tempMatrix1);
         SpecializedOps.addIdentity(tempMatrix1, tempMatrix1, alpha * alpha);
         linearSolver.setA(tempMatrix1);
         tempMatrix2.reshape(A.getNumCols(), b.getNumCols());
         linearSolver.solve(b, tempMatrix2);
         CommonOps.multTransA(A, tempMatrix2, x);
      }
   }

   /** {@inheritDoc}
    * <p>
    *    Finds the inverse of the A matrix set by {@link #setA(DenseMatrix64F)}, accounting for the damping term, alpha. Computes the damped psuedo-inverse by
    * </p>
    * <p>
    *    A_inv = A^T (AA^T + alpha^2)^-1
    * </p>
    */
   @Override
   public void invert(DenseMatrix64F A_inv)
   {
      if (alpha == 0)
      {
         linearSolverAlpha0.setA(this.A);
         linearSolverAlpha0.invert(A_inv);
      }
      else
      {
         tempMatrix1.reshape(A.getNumRows(), A.getNumRows());
         CommonOps.multTransB(A, A, tempMatrix1);
         SpecializedOps.addIdentity(tempMatrix1, tempMatrix1, alpha * alpha);
         linearSolver.setA(tempMatrix1);
         tempMatrix2.reshape(tempMatrix1.getNumRows(), tempMatrix1.getNumCols());
         linearSolver.invert(tempMatrix2);
         CommonOps.multTransA(A, tempMatrix2, A_inv);
      }
   }

   /** {@inheritDoc} */
   @Override
   public boolean modifiesA()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean modifiesB()
   {
      return false;
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public SingularValueDecomposition<DenseMatrix64F> getDecomposition() {
       return null;
   }
}
