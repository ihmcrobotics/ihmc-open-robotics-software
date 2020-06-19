package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DMatrixRMaj;

/**
 * Formulated as Aineq x <= bineq
 */
public class ICPInequalityInput
{
   /** matrix multiplier of the constrained variable. */
   public final DMatrixRMaj Aineq;
   /** desired bounds for the constrained variable. */
   public final DMatrixRMaj bineq;

   private int numberOfConstraints;
   private int numberOfVariables;

   public ICPInequalityInput(int numberOfConstraints, int numberOfVariables)
   {
      this.numberOfConstraints = numberOfConstraints;
      this.numberOfVariables = numberOfVariables;
      Aineq = new DMatrixRMaj(numberOfConstraints, numberOfVariables);
      bineq = new DMatrixRMaj(numberOfConstraints, 1);
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

      this.numberOfConstraints = numberOfConstraints;
      this.numberOfVariables = numberOfVariables;
   }

   /**
    * Returns the number of constraints that this inequality input represents.
    */
   public int getNumberOfConstraints()
   {
      return numberOfConstraints;
   }

   /**
    * Returns the number of variables that are required for this inequality constraint.
    */
   public int getNumberOfVariables()
   {
      return numberOfVariables;
   }
}
