package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;

/**
 * Class that represents any objective task that is submitted to the {@link ICPOptimizationQPSolver}.
 * The intended use is to store the quadratic cost objective, linear cost objective, and scalar cost
 * of a given objective for minimization. This is then added to the full problem quadratic, linear,
 * and scalar costs.
 */
public class ICPQPInput
{
   /** Storage matrix for the quadratic cost of the objective. */
   public DenseMatrix64F quadraticTerm;
   /** Storage matrix for the linear cost of the objective. */
   public DenseMatrix64F linearTerm;
   /** Storage matrix for the scalar cost of the objective. */
   public DenseMatrix64F residualCost = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F tempMatrix;
   private final DenseMatrix64F tempScalar = new DenseMatrix64F(1, 1);

   /**
    * Creates the ICP QP Input. Refer to the class documentation {@link ICPQPInput}.
    * @param size default size for this QP input.
    */
   public ICPQPInput(int size)
   {
      quadraticTerm = new DenseMatrix64F(size, size);
      linearTerm = new DenseMatrix64F(size, 1);
      residualCost = new DenseMatrix64F(1, 1);

      tempMatrix = new DenseMatrix64F(size, 1);
   }

   /**
    * Reshapes the objective input to a new size. In this case, the desired size should be the size of
    * the full problem size for the optimization.
    *
    * @param size new problem size
    */
   public void reshape(int size)
   {
      quadraticTerm.reshape(size, size);
      linearTerm.reshape(size, 1);
   }

   /**
    * Resets the three cost terms in the objective. Should be called before calling {@link #reshape(int)}.
    */
   public void reset()
   {
      quadraticTerm.zero();
      linearTerm.zero();
      residualCost.zero();
   }


   /**
    * Computes the cost of the task given the task value {@param x}.
    */
   public double computeCost(DenseMatrix64F x)
   {
      if (x.getNumRows() != quadraticTerm.getNumRows())
         throw new RuntimeException("x.getNumRows() != quadraticTerms.getNumRows()");

      tempMatrix.reshape(x.getNumRows(), 1);
      tempMatrix.zero();
      tempScalar.zero();

      CommonOps.mult(quadraticTerm, x, tempMatrix);
      CommonOps.multAddTransA(0.5, x, tempMatrix, tempScalar);
      CommonOps.multAddTransA(linearTerm, x, tempScalar);
      CommonOps.addEquals(tempScalar, residualCost);

      return tempScalar.get(0);
   }

   /**
    * Check whether this input is equal to {@param other} within an epsilon of 1e-7.
    */
   public boolean equals(ICPQPInput other)
   {
      return equals(other, 1e-7);
   }

   /**
    * Check whether this input is equal to {@param other} within an epsilon of {@param tol}.
    */
   public boolean equals(ICPQPInput other, double tol)
   {
      return MatrixFeatures.isEquals(quadraticTerm, other.quadraticTerm, tol) && MatrixFeatures.isEquals(linearTerm, other.linearTerm, tol) && MatrixFeatures.isEquals(residualCost, other.residualCost, tol);
   }


}
