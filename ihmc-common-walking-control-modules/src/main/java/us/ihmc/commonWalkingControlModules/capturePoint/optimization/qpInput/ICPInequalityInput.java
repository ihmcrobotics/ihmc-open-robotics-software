package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;

/**
 * Formulated as Aineq x <= bineq
 */
public class ICPInequalityInput
{
   /** matrix multiplier of the constrained variable. */
   public final DenseMatrix64F Aineq;
   /** desired bounds for the constrained variable. */
   public final DenseMatrix64F bineq;

   public ICPInequalityInput(int numberOfConstraints, int size)
   {
      Aineq = new DenseMatrix64F(numberOfConstraints, size);
      bineq = new DenseMatrix64F(numberOfConstraints, 1);

   }

   /**
    * Resets the inequality terms in the bineq. Should be called before calling {@link #reshape(int, int)}.
    */
   public void reset()
   {
      Aineq.zero();
      bineq.zero();
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
      Aineq.reshape(numberOfConstraints, numberOfVariables);
      bineq.reshape(numberOfConstraints, 1);
   }
}
