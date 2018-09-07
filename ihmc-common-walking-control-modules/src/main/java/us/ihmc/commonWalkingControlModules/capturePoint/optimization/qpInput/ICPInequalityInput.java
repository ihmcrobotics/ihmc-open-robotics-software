package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;

/**
 * Formulated as linearOperator x <= objective
 */
public class ICPInequalityInput
{
   public DenseMatrix64F linearOperator;
   public DenseMatrix64F objective;

   public ICPInequalityInput(int numberOfConstraints, int size)
   {
      linearOperator = new DenseMatrix64F(numberOfConstraints, size);
      objective = new DenseMatrix64F(numberOfConstraints, 1);

   }

   /**
    * Resets the inequality terms in the objective. Should be called before calling {@link #reshape(int, int)}.
    */
   public void reset()
   {
      linearOperator.zero();
      objective.zero();
   }

   /**
    * Reshapes the inequality terms to a new size. In this case, the desired size should be the size of
    * the full problem size for the optimization.
    *
    * @param numberOfConstraints new number of constraints that this class is representing.
    * @param numberOfVariables new number of variables in the problem.
    */
   public void reshape(int numberOfConstraints, int numberOfVariables)
   {
      linearOperator.reshape(numberOfConstraints, numberOfVariables);
      objective.reshape(numberOfConstraints, 1);
   }
}
