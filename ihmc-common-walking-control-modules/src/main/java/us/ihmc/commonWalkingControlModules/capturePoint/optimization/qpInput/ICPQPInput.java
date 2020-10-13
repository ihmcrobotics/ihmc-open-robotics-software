package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;

/**
 * Class that represents any objective task that is submitted to the {@link ICPOptimizationQPSolver}.
 * The intended use is to store the quadratic cost objective, linear cost objective, and scalar cost
 * of a given objective for minimization. This is then added to the full problem quadratic, linear,
 * and scalar costs.
 *
 * The problem is formulated as 0.5 x<sup>T</sup> quadraticTerm x - linearTerm<sup>T</sup> x + residualCost
 */
public class ICPQPInput
{
   /** Storage matrix for the quadratic cost of the objective. */
   public DMatrixRMaj quadraticTerm;
   /** Storage matrix for the linear cost of the objective. */
   public DMatrixRMaj linearTerm;
   /** Storage matrix for the scalar cost of the objective. */
   public DMatrixRMaj residualCost;

   private final DMatrixRMaj tempMatrix;
   private final DMatrixRMaj tempScalar = new DMatrixRMaj(1, 1);

   /**
    * Creates the ICP QP Input. Refer to the class documentation {@link ICPQPInput}.
    * @param size default size for this QP input.
    */
   public ICPQPInput(int size)
   {
      quadraticTerm = new DMatrixRMaj(size, size);
      linearTerm = new DMatrixRMaj(size, 1);
      residualCost = new DMatrixRMaj(1, 1);

      tempMatrix = new DMatrixRMaj(size, 1);
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
   public double computeCost(DMatrixRMaj x)
   {
      if (x.getNumRows() != quadraticTerm.getNumRows())
         throw new RuntimeException("x.getNumRows() != quadraticTerms.getNumRows()");

      tempMatrix.reshape(x.getNumRows(), 1);
      tempMatrix.zero();
      tempScalar.zero();

      CommonOps_DDRM.mult(quadraticTerm, x, tempMatrix);
      CommonOps_DDRM.multAddTransA(0.5, x, tempMatrix, tempScalar);
      CommonOps_DDRM.multAddTransA(-1.0, linearTerm, x, tempScalar);
      CommonOps_DDRM.addEquals(tempScalar, residualCost);

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
      return MatrixFeatures_DDRM.isEquals(quadraticTerm, other.quadraticTerm, tol) && MatrixFeatures_DDRM.isEquals(linearTerm, other.linearTerm, tol) && MatrixFeatures_DDRM.isEquals(residualCost, other.residualCost, tol);
   }
}
