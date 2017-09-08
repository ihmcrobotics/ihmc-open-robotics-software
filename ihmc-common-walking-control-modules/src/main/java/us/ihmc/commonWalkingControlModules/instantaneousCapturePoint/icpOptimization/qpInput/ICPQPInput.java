package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPQPOptimizationSolver;

/**
 * Class that represents any objective task that is submitted to the {@link ICPQPOptimizationSolver}.
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

   /**
    * Creates the ICP QP Input. Refer to the class documentation {@link ICPQPInput}.
    * @param size default size for this QP input.
    */
   public ICPQPInput(int size)
   {
      quadraticTerm = new DenseMatrix64F(size, size);
      linearTerm = new DenseMatrix64F(size, 1);
      residualCost = new DenseMatrix64F(1, 1);
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
}
